#include "DQMServices/Core/interface/DQMStore.h"
#include "DQMServices/Core/interface/QReport.h"
#include "DQMServices/Core/interface/QTest.h"
#include "DQMServices/Core/interface/Standalone.h"
#include "DQMServices/Core/src/DQMError.h"
#include "DQMServices/Core/src/ROOTFilePB.pb.h"
#include "TBufferFile.h"
#include "TClass.h"
#include "TFile.h"
#include "TKey.h"
#include "TROOT.h"
#include "TSystem.h"
#include "boost/algorithm/string.hpp"
#include "boost/range/iterator_range_core.hpp"
#include "classlib/utils/Regexp.h"
#include "classlib/utils/RegexpMatch.h"
#include "classlib/utils/StringOps.h"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/gzip_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"

#include <cerrno>
#include <exception>
#include <fstream>
#include <iterator>
#include <sstream>
#include <utility>

/** @var DQMStore::verbose_
    Universal verbose flag for DQM. */

/** @var DQMStore::verboseQT_
    Verbose flag for xml-based QTests. */

/** @var DQMStore::reset_

    Flag used to print out a warning when calling quality tests.
    twice without having called reset() in between; to be reset in
    DQMOldReceiver::runQualityTests.  */

/** @var DQMStore::collateHistograms_ */

/** @var DQMStore::readSelectedDirectory_
    If non-empty, read from file only selected directory. */

/** @var DQMStore::pwd_
    Current directory. */

/** @var DQMStore::qtests_.
    All the quality tests.  */

/** @var DQMStore::qalgos_
    Set of all the available quality test algorithms. */

using MEIdentifier = MonitorElement::Identifier;

namespace {

  //////////////////////////////////////////////////////////////////////
  /// name of global monitoring folder (containing all sources subdirectories)
  std::string const s_empty_string{};
  std::string const s_monitorDirName{"DQMData"};
  std::string const s_referenceDirName{"Reference"};
  std::string const s_collateDirName{"Collate"};
  std::string const s_safe{"/ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwx"
                           "yz0123456789-+=_()# "};

  lat::Regexp const s_rxmeval("^<(.*)>(i|f|s|e|t|qr)=(.*)</\\1>$");
  lat::Regexp const s_rxmeqr1("^st:(\\d+):([-+e.\\d]+):([^:]*):(.*)$");
  lat::Regexp const s_rxmeqr2("^st\\.(\\d+)\\.(.*)$");
  lat::Regexp const s_rxtrace("(.*)\\((.*)\\+0x.*\\).*");
  lat::Regexp const s_rxself("^[^()]*DQMStore::.*");
  lat::Regexp const s_rxpbfile(".*\\.pb$");

  //////////////////////////////////////////////////////////////////////
  /// Check whether the @a path is a subdirectory of @a ofdir.  Returns
  /// true both for an exact match and any nested subdirectory.
  bool isSubdirectory(std::string const& ofdir, std::string const& path)
  {
    return (ofdir.empty() ||
            (path.size() >= ofdir.size() &&
             path.compare(0, ofdir.size(), ofdir) == 0 &&
             (path.size() == ofdir.size() || path[ofdir.size()] == '/')));
  }

  void cleanTrailingSlashes(std::string const& path, std::string& clean,
                            std::string const*& cleaned)
  {
    clean.clear();
    cleaned = &path;

    size_t len = path.size();
    for (; len > 0 && path[len - 1] == '/'; --len)
      ;

    if (len != path.size()) {
      clean = path.substr(0, len);
      cleaned = &clean;
    }
  }

  void splitPath(std::string& dir, std::string& name, std::string const& path)
  {
    size_t slash = path.rfind('/');
    if (slash != std::string::npos) {
      dir.append(path, 0, slash);
      name.append(path, slash + 1, std::string::npos);
    }
    else
      name = path;
  }

  void mergePath(std::string& path,
                 std::string const& dir,
                 std::string const& name)
  {
    path.reserve(dir.size() + name.size() + 2);
    path += dir;
    if (!path.empty())
      path += '/';
    path += name;
  }

  template <class T>
  void initQCriterion(
      std::map<std::string, std::function<QCriterion*(std::string const&)>>& m)
  {
    m[T::getAlgoName()] = [](std::string const& qtname) {
      return new T(qtname);
    };
  }

} // anon. namespace

/////////////////////////////////////////////////////////////
fastmatch::fastmatch(std::string const& fastString)
    : fastString_{fastString}, matching_{UseFull}
{
  try {
    regexp_ =
        std::make_unique<lat::Regexp>(fastString_, 0, lat::Regexp::Wildcard);
    regexp_->study();
  }
  catch (lat::Error& e) {
    raiseDQMError("DQMStore", "Invalid wildcard pattern '%s' in quality"
                              " test specification",
                  fastString_.c_str());
  }

  // count stars ( "*" )
  size_t starCount{};
  int pos = -1;
  while (true) {
    pos = fastString_.find("*", pos + 1);
    if (static_cast<size_t>(pos) == std::string::npos)
      break;
    ++starCount;
  }

  // investigate for heuristics
  if ((fastString_.find('"') != std::string::npos) ||
      (fastString_.find(']') != std::string::npos) ||
      (fastString_.find('?') != std::string::npos) ||
      (fastString_.find('\\') != std::string::npos) || (starCount > 2)) {
    // no fast version can be used
    return;
  }

  // match for pattern "*MyString" and "MyString*"
  if (starCount == 1) {
    if (boost::algorithm::starts_with(fastString_, "*")) {
      matching_ = OneStarStart;
      fastString_.erase(0, 1);
      return;
    }

    if (boost::algorithm::ends_with(fastString_, "*")) {
      matching_ = OneStarEnd;
      fastString_.erase(fastString_.length() - 1, 1);
      return;
    }
  }

  // match for pattern "*MyString*"
  if (starCount == 2) {
    if (boost::algorithm::starts_with(fastString_, "*") &&
        boost::algorithm::ends_with(fastString_, "*")) {
      matching_ = TwoStar;
      fastString_.erase(0, 1);
      fastString_.erase(fastString_.size() - 1, 1);
      return;
    }
  }
}

bool fastmatch::compare_strings_reverse(std::string const& pattern,
                                        std::string const& input) const
{
  if (input.size() < pattern.size())
    return false;

  // compare the two strings character by character for equalness:
  // this does not create uneeded copies of std::string. The
  // boost::algorithm implementation does
  auto rit_pattern = pattern.crbegin();
  auto rit_input = input.crbegin();

  for (; rit_pattern < pattern.rend(); rit_pattern++, rit_input++) {
    if (*rit_pattern != *rit_input)
      // found a difference, fail
      return false;
  }
  return true;
}

bool fastmatch::compare_strings(std::string const& pattern,
                                std::string const& input) const
{
  if (input.size() < pattern.size())
    return false;

  // compare the two strings character by character for equalness:
  // this does not create uneeded copies of std::string. The
  // boost::algorithm implementation does.
  auto rit_pattern = pattern.cbegin();
  auto rit_input = input.cbegin();

  for (; rit_pattern < pattern.end(); rit_pattern++, rit_input++) {
    if (*rit_pattern != *rit_input)
      // found a difference, fail
      return false;
  }
  return true;
}

bool fastmatch::match(std::string const& s) const
{
  switch (matching_) {
  case OneStarStart:
    return compare_strings_reverse(fastString_, s);

  case OneStarEnd:
    return compare_strings(fastString_, s);

  case TwoStar:
    return (s.find(fastString_) != std::string::npos);

  default:
    return regexp_->match(s);
  }
}

// IBooker methods
void DQMStore::IBooker::cd() { owner_->cd(); }

void DQMStore::IBooker::cd(std::string const& dir) { owner_->cd(dir); }

void DQMStore::IBooker::setCurrentFolder(std::string const& fullpath)
{
  owner_->setCurrentFolder(fullpath);
}

void DQMStore::IBooker::goUp() { owner_->goUp(); }

std::string const&  DQMStore::IBooker::pwd()
{
  return owner_->pwd();
}

void DQMStore::IBooker::tag(MonitorElement* me, unsigned int tag)
{
  owner_->tag(me, tag);
}

void DQMStore::IBooker::tagContents(std::string const& path, unsigned int const myTag)
{
  owner_->tagContents(path, myTag);
}

// IGetter methods
std::vector<MonitorElement*>
DQMStore::IGetter::getAllContents(std::string const& path,
                                  uint32_t const runNumber /* = 0 */,
                                  uint32_t const lumi /* = 0 */)
{
  return owner_->getAllContents(path, runNumber, lumi);
}

MonitorElement* DQMStore::IGetter::get(std::string const& path)
{
  return owner_->get(path);
}

MonitorElement* DQMStore::IGetter::getElement(std::string const& path)
{
  MonitorElement* ptr{get(path)};
  if (ptr == nullptr) {
    std::stringstream msg;
    msg << "DQM object not found";
    msg << ": " << path;

    // can't use cms::Exception inside DQMStore
    throw std::out_of_range(msg.str());
  }
  return ptr;
}

std::vector<std::string> DQMStore::IGetter::getSubdirs()
{
  return owner_->getSubdirs();
}

std::vector<std::string> DQMStore::IGetter::getMEs()
{
  return owner_->getMEs();
}

bool DQMStore::IGetter::containsAnyMonitorable(std::string const& path)
{
  return owner_->containsAnyMonitorable(path);
}

bool DQMStore::IGetter::dirExists(std::string const& path)
{
  return owner_->dirExists(path);
}

void DQMStore::IGetter::cd() { owner_->cd(); }

void DQMStore::IGetter::cd(std::string const& dir) { owner_->cd(dir); }

void DQMStore::IGetter::setCurrentFolder(std::string const& fullpath)
{
  owner_->setCurrentFolder(fullpath);
}

/** Function to transfer the local copies of histograms from each
    stream into the global ROOT Object. Since this involves de-facto a
    booking action in the case in which the global object is not yet
    there, the function requires the acquisition of the central lock
    into the DQMStore.
    In case we book the global object for the first time, no Add action is
    needed since the ROOT histograms is cloned starting from the local
    one. */

void DQMStore::mergeAndResetMEsRunSummaryCache(uint32_t const run,
                                               uint32_t const streamId,
                                               uint32_t const moduleId)
{
  if (verbose_ > 1) {
    std::cout << "DQMStore::mergeAndResetMEsRunSummaryCache - Merging objects "
                 "from run: "
              << run << ", stream: " << streamId << " module: " << moduleId
              << std::endl;
  }

  if (LSbasedMode_) {
    return;
  }

  MonitorElement const proto {&s_empty_string, s_empty_string, run, streamId, moduleId};
  // Since this accesses the data, the operation must be
  // be locked.
  std::lock_guard<std::mutex> guard(book_mutex_);

  auto e = data_.end();
  auto i = data_.lower_bound(proto);
  while (i != e) {
    if (i->data_.run != run
        || i->data_.streamId != streamId
        || i->data_.moduleId != moduleId)
      break;

    // Handle Run-based histograms only.
    if (i->getLumiFlag() || LSbasedMode_) {
      ++i;
      continue;
    }

    // don't call the copy constructor
    // we are just searching for a global histogram - a copy is not necessary
    MonitorElement global_me{*i, MonitorElementNoCloneTag()};
    global_me.globalize();

    auto me = data_.find(global_me);
    if (me != data_.end()) {
      if (verbose_ > 1) {
        std::cout << "Found global Object, using it --> " << me->getFullname()
                  << std::endl;
      }

      // don't take any action if the ME is an INT || FLOAT || STRING
      if (me->kind() >= MonitorElement::DQM_KIND_TH1F) {
        if (me->getTH1()->CanExtendAllAxes() &&
            i->getTH1()->CanExtendAllAxes()) {
          TList list;
          list.Add(i->getTH1());
          if (-1 == me->getTH1()->Merge(&list)) {
            std::cout << "mergeAndResetMEsRunSummaryCache: Failed to merge DQM "
                         "element "
                      << me->getFullname();
          }
        }
        else if (i->getTH1()->GetEntries()) {
          me->getTH1()->Add(i->getTH1());
        }
      }
    }
    else {
      if (verbose_ > 1) {
        std::cout << "No global Object found. " << std::endl;
      }

      // this makes an actual and a single copy with Clone()'ed th1
      MonitorElement actual_global_me(*i);
      actual_global_me.globalize();
      actual_global_me.markToDelete();
      //      safe_data_[KeyType{streamId, moduleId}].insert(actual_global_me);
      auto gme [[gnu::unused]] = data_.insert(std::move(actual_global_me));
    }
    // TODO(rovere): eventually reset the local object and mark it as reusable??
    ++i;
  }
}

void DQMStore::mergeAndResetMEsLuminositySummaryCache(uint32_t const run,
                                                      uint32_t const lumi,
                                                      uint32_t const streamId,
                                                      uint32_t const moduleId)
{
  if (verbose_ > 1) {
    std::cout << "DQMStore::mergeAndResetMEsLuminositySummaryCache - Merging "
                 "objects from run: "
              << run << " lumi: " << lumi << ", stream: " << streamId
              << " module: " << moduleId << std::endl;
  }

  MonitorElement const proto {&s_empty_string, s_empty_string, run, streamId, moduleId};
  // Since this accesses the data, the operation must be
  // be locked.
  std::lock_guard<std::mutex> guard(book_mutex_);

  auto e = data_.end();
  auto i = data_.lower_bound(proto);
  for (; i != e; ++i) {
    if (i->data_.run != run
        || i->data_.streamId != streamId
        || i->data_.moduleId != moduleId)
      break;

    // Handle LS-based histograms only.
    if (not(i->getLumiFlag() || LSbasedMode_)) {
      continue;
    }

    MonitorElement global_me{*i, MonitorElementNoCloneTag()};
    global_me.globalize();
    global_me.setLumi(lumi);
    auto me = data_.find(global_me);
    if (me != data_.end()) {
      if (verbose_ > 1)
        std::cout << "Found global Object, using it --> " << me->getFullname()
                  << std::endl;

      // don't take any action if the ME is an INT || FLOAT || STRING
      if (me->kind() >= MonitorElement::DQM_KIND_TH1F) {
        if (me->getTH1()->CanExtendAllAxes() &&
            i->getTH1()->CanExtendAllAxes()) {
          TList list;
          list.Add(i->getTH1());
          if (-1 == me->getTH1()->Merge(&list)) {
            std::cout << "mergeAndResetMEsLuminositySummaryCache: Failed to "
                         "merge DQM element "
                      << me->getFullname();
          }
        }
        else if (i->getTH1()->GetEntries()) {
          me->getTH1()->Add(i->getTH1());
        }
      }
    }
    else {
      if (verbose_ > 1) {
        std::cout << "No global Object found. " << std::endl;
      }

      // this makes an actual and a single copy with Clone()'ed th1
      MonitorElement actual_global_me(*i);
      actual_global_me.globalize();
      actual_global_me.setLumi(lumi);
      actual_global_me.markToDelete();
      //      safe_data_[KeyType{streamId,moduleId}].insert(actual_global_me);
      auto gme [[gnu::unused]] = data_.insert(std::move(actual_global_me));
      assert(gme.second);
    }
    // make the ME reusable for the next LS
    const_cast<MonitorElement*>(&*i)->Reset();
  }
}

//////////////////////////////////////////////////////////////////////
DQMStore::DQMStore(edm::ParameterSet const& pset, edm::ActivityRegistry& ar)
    : DQMStore{pset}
{
  if (pset.getUntrackedParameter<bool>("forceResetOnBeginRun", false)) {
    ar.watchPostSourceRun(this, &DQMStore::forceReset);
  }
  ar.preallocateSignal_.connect(
      [this](edm::service::SystemBounds const& iBounds) {
        if (iBounds.maxNumberOfStreams() > 1) {
          enableMultiThread_ = true;
        }
      });
  if (pset.getUntrackedParameter<bool>("forceResetOnBeginLumi", false) &&
      enableMultiThread_ == false) {
    forceResetOnBeginLumi_ = true;
    ar.watchPostSourceLumi(this, &DQMStore::forceReset);
  }
  ar.watchPostGlobalBeginLumi(this, &DQMStore::postGlobalBeginLumi);
}

DQMStore::DQMStore(edm::ParameterSet const& pset)
{
  initializeFrom(pset);
}

DQMStore::~DQMStore()
{
  for (auto& q : qtests_)
    delete q.second;

  for (auto& qt : qtestspecs_)
    delete qt.first;

  if (stream_)
    stream_->close();
  delete stream_;
}

void DQMStore::initializeFrom(edm::ParameterSet const& pset)
{
  makeDirectory("");
  reset();

  // set steerable parameters
  verbose_ = pset.getUntrackedParameter<int>("verbose", 0);
  if (verbose_ > 0)
    std::cout << "DQMStore: verbosity set to " << verbose_ << std::endl;

  verboseQT_ = pset.getUntrackedParameter<int>("verboseQT", 0);
  if (verbose_ > 0)
    std::cout << "DQMStore: QTest verbosity set to " << verboseQT_ << std::endl;

  collateHistograms_ =
      pset.getUntrackedParameter<bool>("collateHistograms", false);
  if (collateHistograms_)
    std::cout << "DQMStore: histogram collation is enabled\n";

  enableMultiThread_ =
      pset.getUntrackedParameter<bool>("enableMultiThread", false);
  if (enableMultiThread_)
    std::cout << "DQMStore: MultiThread option is enabled\n";

  LSbasedMode_ = pset.getUntrackedParameter<bool>("LSbasedMode", false);
  if (LSbasedMode_)
    std::cout << "DQMStore: LSbasedMode option is enabled\n";

  std::string ref =
      pset.getUntrackedParameter<std::string>("referenceFileName", "");
  if (!ref.empty()) {
    std::cout << "DQMStore: using reference file '" << ref << "'\n";
    readFile(ref, true, "", s_referenceDirName, StripRunDirs, false);
  }

  initQCriterion<Comp2RefChi2>(qalgos_);
  initQCriterion<Comp2Ref2DChi2>(qalgos_);
  initQCriterion<Comp2RefKolmogorov>(qalgos_);
  initQCriterion<ContentsXRange>(qalgos_);
  initQCriterion<ContentsYRange>(qalgos_);
  initQCriterion<MeanWithinExpected>(qalgos_);
  initQCriterion<Comp2RefEqualH>(qalgos_);
  initQCriterion<DeadChannel>(qalgos_);
  initQCriterion<NoisyChannel>(qalgos_);
  initQCriterion<ContentsWithinExpected>(qalgos_);
  initQCriterion<CompareToMedian>(qalgos_);
  initQCriterion<CompareLastFilledBin>(qalgos_);
  initQCriterion<CheckVariance>(qalgos_);

  scaleFlag_ = pset.getUntrackedParameter<double>("ScalingFlag", 0.0);
  if (verbose_ > 0)
    std::cout << "DQMStore: Scaling Flag set to " << scaleFlag_ << std::endl;
}

/* Generic method to do a backtrace and print it to stdout. It is
   customised to properly get the routine that called the booking of the
   histograms, which, following the usual stack, is at position 4. The
   name of the calling function is properly demangled and the original
   shared library including this function is also printed. For a more
   detailed explanation of the routines involved, see here:
   http://www.gnu.org/software/libc/manual/html_node/Backtraces.html
   http://gcc.gnu.org/onlinedocs/libstdc++/manual/ext_demangling.html.*/

void DQMStore::print_trace(std::string const& dir, std::string const& name)
{
  // the access to the member stream_ is implicitly protected against
  // concurrency problems because the print_trace method is always
  // called behind a lock (see bookTransaction).
  if (!stream_)
    stream_ = new std::ofstream("histogramBookingBT.log");

  void* array[10];
  size_t size;
  char** strings;
  int r = 0;
  lat::RegexpMatch m;
  m.reset();

  size = backtrace(array, 10);
  strings = backtrace_symbols(array, size);

  size_t level = 1;
  char* demangled = nullptr;
  for (; level < size; level++) {
    if (!s_rxtrace.match(strings[level], 0, 0, &m)) continue;
    demangled = abi::__cxa_demangle(m.matchString(strings[level], 2).c_str(), nullptr, nullptr, &r);
    if (!demangled) continue;
    if (!s_rxself.match(demangled, 0, 0)) break;
    free(demangled);
    demangled = nullptr;
  }

  if (demangled != nullptr) {
    *stream_ << "\"" << dir << "/" << name << "\" "
             << (r ? m.matchString(strings[level], 2) : demangled) << " "
             << m.matchString(strings[level], 1) << "\n";
    free(demangled);
  }
  else {
    *stream_ << "Skipping " << dir << "/" << name << " with stack size " << size
             << "\n";
  }

  /* In this case print the full stack trace, up to main or to the
   * maximum stack size, i.e. 10. */
  if (verbose_ > 4 || demangled == nullptr) {
    size_t i;
    m.reset();

    for (i = 0; i < size; i++)
      if (s_rxtrace.match(strings[i], 0, 0, &m)) {
        char* demangled =
            abi::__cxa_demangle(m.matchString(strings[i], 2).c_str(), 0, 0, &r);
        *stream_ << "\t\t" << i << "/" << size << " "
                 << (r ? m.matchString(strings[i], 2) : demangled) << " "
                 << m.matchString(strings[i], 1) << std::endl;
        free(demangled);
      }
  }
  free(strings);
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/// set verbose level (0 turns all non-error messages off)
void DQMStore::setVerbose(unsigned /* level */) {}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/// return pathname of current directory
std::string const& DQMStore::pwd() const { return pwd_; }

/// go to top directory (ie. root)
void DQMStore::cd() { setCurrentFolder(""); }

/// cd to subdirectory (if there)
void DQMStore::cd(std::string const& subdir)
{
  std::string clean;
  std::string const* cleaned{nullptr};
  cleanTrailingSlashes(subdir, clean, cleaned);

  if (!dirExists(*cleaned))
    raiseDQMError("DQMStore", "Cannot 'cd' into non-existent directory '%s'",
                  cleaned->c_str());

  setCurrentFolder(*cleaned);
}

/// set the last directory in fullpath as the current directory(create if
/// needed);
/// to be invoked by user to specify directories for monitoring objects
/// before booking;
/// commands book1D (etc) & removeElement(name) imply elements in this
/// directory!;
void DQMStore::setCurrentFolder(std::string const& fullpath)
{
  std::string clean;
  std::string const* cleaned{nullptr};
  cleanTrailingSlashes(fullpath, clean, cleaned);
  makeDirectory(*cleaned);
  pwd_ = *cleaned;
}

/// equivalent to "cd .."
void DQMStore::goUp()
{
  size_t const pos{pwd_.rfind('/')};
  if (pos == std::string::npos)
    setCurrentFolder("");
  else
    setCurrentFolder(pwd_.substr(0, pos));
}

// -------------------------------------------------------------------
/// get folder corresponding to inpath wrt to root (create subdirs if
/// necessary)
void DQMStore::makeDirectory(std::string const& path) // PRIVATE
{
  std::string prev;
  std::string subdir;
  std::string name;
  prev.reserve(path.size());
  subdir.reserve(path.size());
  name.reserve(path.size());
  size_t prevname = 0;
  size_t slash = 0;

  while (true) {
    // Create this subdirectory component.
    subdir.clear();
    subdir.append(path, 0, slash);
    name.clear();
    name.append(subdir, prevname, std::string::npos);
    if (!prev.empty() && findObject(prev, name))
      raiseDQMError("DQMStore", "Attempt to create subdirectory '%s'"
                                " which already exists as a monitor element",
                    subdir.c_str());

    if (!dirs_.count(subdir))
      dirs_.insert(subdir);

    // Stop if we've reached the end (including possibly a trailing slash).
    if (slash + 1 >= path.size())
      break;

    // Find the next slash, making sure we progress.  If reach the end,
    // process the last path component; the next loop round will terminate.
    prevname = slash ? slash + 1 : slash;
    prev = subdir;
    if ((slash = path.find('/', ++slash)) == std::string::npos)
      slash = path.size();
  }
}

/// true if directory exists
bool DQMStore::dirExists(std::string const& path) const
{
  return dirs_.count(path) > 0;
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
template <class HISTO, class COLLATE>
MonitorElement*
DQMStore::book_(std::string const& dir,
                std::string const& name,
                char const* context,
                int const kind,
                HISTO* h,
                COLLATE collate,
                MEIdentifier const& id)
{
  assert(name.find('/') == std::string::npos);
  // NOT THREAD-SAFE
  if (verbose_ > 3)
    print_trace(dir, name);

  std::string path;
  mergePath(path, dir, name);

  // Put us in charge of h.
  h->SetDirectory(nullptr);

  // Check if the request monitor element already exists.
  auto me = findObject(dir, name, id);
  if (me) {
    if (collateHistograms_) {
      collate(me, h, verbose_);
    }
    else {
      if (verbose_ > 1) {
        std::cout << "DQMStore: " << context << ": monitor element '" << path
                  << "' already exists, collating" << std::endl;
      }
      me->Reset();
      collate(me, h, verbose_);
    }
    delete h;
    return me;
  }
  else {
    // Create and initialise core object.
    assert(dirs_.count(dir));
    MonitorElement proto{&*dirs_.find(dir), name, id.run, id.streamId, id.moduleId};
    auto const& result = *data_.insert(std::move(proto)).first;
    //    auto const& result = *safe_data_[KeyType{id.streamId,id.moduleId}].insert(std::move(proto)).first;
    me = const_cast<MonitorElement&>(result).initialise((MonitorElement::Kind)kind, h);

    // Initialise quality test information.
    for (auto const& q : qtestspecs_) {
      if (q.first->match(path))
        me->addQReport(q.second);
    }

    // If we just booked a (plain) MonitorElement, and there is a reference
    // MonitorElement with the same name, link the two together.
    // The other direction is handled by the extract method.
    std::string refdir;
    refdir.reserve(s_referenceDirName.size() + dir.size() + 1);
    refdir += s_referenceDirName;
    refdir += '/';
    refdir += dir;
    if (auto referenceME = findObject(refdir, name)) {
      // We have booked a new MonitorElement with a specific dir and name.
      // Then, if we can find the corresponding MonitorElement in the reference
      // dir we assign the object_ of the reference MonitorElement to the
      // reference_ property of our new MonitorElement.
      me->data_.flags |= DQMNet::DQM_PROP_HAS_REFERENCE;
      me->reference_ = referenceME->object_;
    }

    // Return the monitor element.
    return me;
  }
}

MonitorElement*
DQMStore::book_(std::string const& dir,
                std::string const& name,
                const char* context,
                MEIdentifier const& id)
{
  assert(name.find('/') == std::string::npos);

  // Not thread safe
  if (verbose_ > 3)
    print_trace(dir, name);

  // Check if the request monitor element already exists.
  if (auto me = findObject(dir, name, id)) {
    if (verbose_ > 1) {
      std::string path;
      mergePath(path, dir, name);

      std::cout << "DQMStore: " << context << ": monitor element '" << path
                << "' already exists, resetting" << std::endl;
    }
    me->Reset();
    return me;
  }

  // Create it and return for initialisation.
  assert(dirs_.count(dir));
  MonitorElement proto{&*dirs_.find(dir), name, id.run, id.streamId, id.moduleId};
  //  auto const& result = *safe_data_[KeyType{id.streamId, id.moduleId}].insert(std::move(proto)).first;
  auto const& result = *data_.insert(std::move(proto)).first;
  return &const_cast<MonitorElement&>(result);
}

// -------------------------------------------------------------------
/// Book int.
MonitorElement*
DQMStore::bookInt_(std::string const& dir, std::string const& name,
                   MEIdentifier const& id)
{
  if (collateHistograms_) {
    if (auto me = findObject(dir, name, id)) {
      me->Fill(0);
      return me;
    }
  }

  return book_(dir, name, "bookInt", id)
    ->initialise(MonitorElement::DQM_KIND_INT);
}

/// Book int.
MonitorElement* DQMStore::bookInt(std::string const& name, MEIdentifier const& id)
{
  return bookInt_(pwd_, name, id);
}

// -------------------------------------------------------------------
/// Book float.
MonitorElement*
DQMStore::bookFloat_(std::string const& dir,
                     std::string const& name,
                     MEIdentifier const& id)
{
  if (collateHistograms_) {
    if (auto me = findObject(dir, name, id)) {
      me->Fill(0.);
      return me;
    }
  }

  return book_(dir, name, "bookFloat", id)
    ->initialise(MonitorElement::DQM_KIND_REAL);
}

/// Book float.
MonitorElement* DQMStore::bookFloat(std::string const& name,
                                    MEIdentifier const& id)
{
  return bookFloat_(pwd_, name, id);
}

// -------------------------------------------------------------------
/// Book string.
MonitorElement *
DQMStore::bookString_(std::string const& dir,
                      std::string const& name,
                      std::string const& value,
                      MEIdentifier const& id)
{
  if (collateHistograms_) {
    if (auto me = findObject(dir, name, id))
      return me;
  }

  return book_(dir, name, "bookString", id)
    ->initialise(MonitorElement::DQM_KIND_STRING, value);
}

/// Book string.
MonitorElement* DQMStore::bookString(std::string const& name,
                                     std::string const& value,
                                     MEIdentifier const& id)
{
  return bookString_(pwd_, name, value, id);
}

// -------------------------------------------------------------------
/// Book 1S histogram.
MonitorElement*
DQMStore::book1S(char_string const& name, char_string const& title,
                 int nchX, double lowX, double highX, MEIdentifier const& id)
{
  return book1S_(pwd_, name, new TH1S{name, title, nchX, lowX, highX}, id);
}

/// Book 1S histogram based on TH1S.
MonitorElement *
DQMStore::book1S_(std::string const& dir, std::string const& name, TH1S *h, MEIdentifier const& id)
{
  return book_(dir, name, "book1S", MonitorElement::DQM_KIND_TH1S, h, collate1S, id);
}

/// Book 1S histogram by cloning an existing histogram.
MonitorElement *
DQMStore::book1S(char_string const& name, TH1S *source, MEIdentifier const& id)
{
  return book1S_(pwd_, name, static_cast<TH1S *>(source->Clone(name)), id);
}

/// Book 1D histogram based on TH1F.
MonitorElement *
DQMStore::book1D_(std::string const& dir, std::string const& name, TH1F *h, MEIdentifier const& id)
{
  return book_(dir, name, "book1D", MonitorElement::DQM_KIND_TH1F, h, collate1D, id);
}

/// Book 1D histogram based on TH1D.
MonitorElement *
DQMStore::book1DD_(std::string const& dir, std::string const& name, TH1D *h, MEIdentifier const& id)
{
  return book_(dir, name, "book1DD", MonitorElement::DQM_KIND_TH1D, h, collate1DD, id);
}

/// Book 1D histogram.
MonitorElement *
DQMStore::book1D(char_string const& name, char_string const& title,
                 int nchX, double lowX, double highX, MEIdentifier const& id)
{
  return book1D_(pwd_, name, new TH1F{name, title, nchX, lowX, highX}, id);
}

/// Book 1D variable bin histogram.
MonitorElement*
DQMStore::book1D(char_string const& name, char_string const& title,
                 int nchX, const float* xbinsize, MEIdentifier const& id)
{
  return book1D_(pwd_, name, new TH1F{name, title, nchX, xbinsize}, id);
}

/// Book 1D histogram by cloning an existing histogram.
MonitorElement *
DQMStore::book1D(char_string const& name, TH1F *source, MEIdentifier const& id)
{
  return book1D_(pwd_, name, static_cast<TH1F*>(source->Clone(name)), id);
}

/// Book 1D double histogram by cloning an existing histogram.
MonitorElement*
DQMStore::book1DD(char_string const& name, TH1D* const source, MEIdentifier const& id)
{
  return book1DD_(pwd_, name, static_cast<TH1D *>(source->Clone(name)), id);
}

// -------------------------------------------------------------------
/// Book 2D histogram based on TH2F.
MonitorElement *
DQMStore::book2D_(std::string const& dir, std::string const& name, TH2F *h, MEIdentifier const& id)
{
  return book_(dir, name, "book2D", MonitorElement::DQM_KIND_TH2F, h, collate2D, id);
}

/// Book 2D histogram based on TH2S.
MonitorElement *
DQMStore::book2S_(std::string const& dir, std::string const& name, TH2S *h, MEIdentifier const& id)
{
  return book_(dir, name, "book2S", MonitorElement::DQM_KIND_TH2S, h, collate2S, id);
}

/// Book 2D histogram based on TH2D.
MonitorElement *
DQMStore::book2DD_(std::string const& dir, std::string const& name, TH2D *h, MEIdentifier const& id)
{
  return book_(dir, name, "book2DD", MonitorElement::DQM_KIND_TH2D, h, collate2DD, id);
}

/// Book 2D histogram.
MonitorElement* DQMStore::book2D(char_string const& name,
                                 char_string const& title, int nchX,
                                 double lowX, double highX, int nchY,
                                 double lowY, double highY, MEIdentifier const& id)
{
  return book2D_(pwd_, name, new TH2F{name, title,
        nchX, lowX, highX,
        nchY, lowY, highY}, id);
}

/// Book 2S histogram.
MonitorElement* DQMStore::book2S(char_string const& name,
                                 char_string const& title, int nchX,
                                 double lowX, double highX, int nchY,
                                 double lowY, double highY, MEIdentifier const& id)
{
  return book2S_(pwd_, name, new TH2S(name, title,
                                      nchX, lowX, highX,
                                      nchY, lowY, highY), id);
}

/// Book 2D double histogram.
MonitorElement* DQMStore::book2DD(char_string const& name,
                                  char_string const& title, int nchX,
                                  double lowX, double highX, int nchY,
                                  double lowY, double highY, MEIdentifier const& id)
{
  return book2DD_(pwd_, name, new TH2D(name, title,
                                       nchX, lowX, highX,
                                       nchY, lowY, highY), id);
}

/// Book 2D variable bin histogram.
MonitorElement* DQMStore::book2D(char_string const& name,
                                 char_string const& title, int nchX,
                                 const float* xbinsize, int nchY,
                                 const float* ybinsize, MEIdentifier const& id)
{
    return book2D_(pwd_, name, new TH2F{name, title,
          nchX, xbinsize, nchY, ybinsize}, id);
}

/// Book 2S variable bin histogram.
MonitorElement *
DQMStore::book2S(char_string const& name, char_string const& title,
                 int nchX, const float *xbinsize, int nchY, const float *ybinsize, MEIdentifier const& id)
{
  return book2S_(pwd_, name, new TH2S(name, title,
                                      nchX, xbinsize, nchY, ybinsize), id);
}

/// Book 2D histogram by cloning an existing histogram.
MonitorElement*
DQMStore::book2D(char_string const& name, TH2F* source, MEIdentifier const& id)
{
  return book2D_(pwd_, name, static_cast<TH2F*>(source->Clone(name)), id);
}

/// Book 2DS histogram by cloning an existing histogram.
MonitorElement*
DQMStore::book2S(char_string const& name, TH2S* source, MEIdentifier const& id)
{
  return book2S_(pwd_, name, static_cast<TH2S*>(source->Clone(name)), id);
}

/// Book 2DS histogram by cloning an existing histogram.
MonitorElement*
DQMStore::book2DD(char_string const& name, TH2D* source, MEIdentifier const& id)
{
  return book2DD_(pwd_, name, static_cast<TH2D*>(source->Clone(name)), id);
}

// -------------------------------------------------------------------
/// Book 3D histogram based on TH3F.
MonitorElement *
DQMStore::book3D_(std::string const& dir, std::string const& name, TH3F *h, MEIdentifier const& id)
{
  return book_(dir, name, "book3D", MonitorElement::DQM_KIND_TH3F, h, collate3D, id);
}

/// Book 3D histogram.
MonitorElement* DQMStore::book3D(char_string const& name,
                                 char_string const& title, int nchX,
                                 double lowX, double highX, int nchY,
                                 double lowY, double highY, int nchZ,
                                 double lowZ, double highZ, MEIdentifier const& id)
{
  return book3D_(pwd_, name, new TH3F(name, title,
                                      nchX, lowX, highX,
                                      nchY, lowY, highY,
                                      nchZ, lowZ, highZ), id);
}

/// Book 3D histogram by cloning an existing histogram.
MonitorElement*
DQMStore::book3D(char_string const& name, TH3F* source, MEIdentifier const& id)
{
  return book3D_(pwd_, name, static_cast<TH3F *>(source->Clone(name)), id);
}

// -------------------------------------------------------------------
/// Book profile histogram based on TProfile.
MonitorElement *
DQMStore::bookProfile_(std::string const& dir, std::string const& name, TProfile *h, MEIdentifier const& id)
{
  return book_(dir, name, "bookProfile",
               MonitorElement::DQM_KIND_TPROFILE,
               h, collateProfile, id);
}

/// Book profile.  Option is one of: " ", "s" (default), "i", "G" (see
/// TProfile::BuildOptions).  The number of channels in Y is
/// disregarded in a profile plot.
MonitorElement* DQMStore::bookProfile(char_string const& name,
                                      char_string const& title, int nchX,
                                      double lowX, double highX, int /* nchY */,
                                      double lowY, double highY,
                                      char const* option /* = "s" */, MEIdentifier const& id)
{
  return bookProfile_(pwd_, name, new TProfile(name, title,
                                               nchX, lowX, highX,
                                               lowY, highY,
                                               option), id);
}

/// Book profile.  Option is one of: " ", "s" (default), "i", "G" (see
/// TProfile::BuildOptions).  The number of channels in Y is
/// disregarded in a profile plot.
MonitorElement*
DQMStore::bookProfile(char_string const& name, char_string const& title,
                      int nchX, double lowX, double highX,
                      double lowY, double highY,
                      const char* option /* = "s" */,
                      MEIdentifier const& id)
{
  return bookProfile_(pwd_, name, new TProfile(name, title,
                                               nchX, lowX, highX,
                                               lowY, highY,
                                               option), id);
}

/// Book variable bin profile.  Option is one of: " ", "s" (default), "i", "G"
/// (see
/// TProfile::BuildOptions).  The number of channels in Y is
/// disregarded in a profile plot.
MonitorElement*
DQMStore::bookProfile(char_string const& name, char_string const& title,
                      int nchX, const double* xbinsize,
                      int /* nchY */, double lowY, double highY,
                      const char* option /* = "s" */,
                      MEIdentifier const& id)
{
  return bookProfile_(pwd_, name, new TProfile(name, title,
                                               nchX, xbinsize,
                                               lowY, highY,
                                               option), id);
}

/// Book variable bin profile.  Option is one of: " ", "s" (default), "i", "G"
/// (see
/// TProfile::BuildOptions).  The number of channels in Y is
/// disregarded in a profile plot.
MonitorElement*
DQMStore::bookProfile(char_string const& name,
                      char_string const& title, int nchX,
                      const double* xbinsize, double lowY,
                      double highY,
                      char const* option /* = "s" */,
                      MEIdentifier const& id)
{
  return bookProfile_(pwd_, name, new TProfile(name, title, nchX, xbinsize, lowY,
                                               highY, option), id);
}

/// Book TProfile by cloning an existing profile.
MonitorElement*
DQMStore::bookProfile(char_string const& name, TProfile* source, MEIdentifier const& id)
{
  return bookProfile_(pwd_, name, static_cast<TProfile*>(source->Clone(name)), id);
}

// -------------------------------------------------------------------
/// Book 2D profile histogram based on TProfile2D.
MonitorElement *
DQMStore::bookProfile2D_(std::string const& dir, const std::string &name, TProfile2D *h, MEIdentifier const& id)
{
  return book_(dir, name, "bookProfile2D",
               MonitorElement::DQM_KIND_TPROFILE2D,
               h, collateProfile2D, id);
}

/// Book 2-D profile.  Option is one of: " ", "s" (default), "i", "G"
/// (see TProfile2D::BuildOptions).  The number of channels in Z is
/// disregarded in a 2-D profile.
MonitorElement*
DQMStore::bookProfile2D(char_string const& name, char_string const& title,
                        int nchX, double lowX, double highX, int nchY,
                        double lowY, double highY, int /* nchZ */, double lowZ,
                        double highZ, char const* option /* = "s" */, MEIdentifier const& id)
{
  return bookProfile2D_(pwd_, name, new TProfile2D(name, title,
                                                   nchX, lowX, highX,
                                                   nchY, lowY, highY,
                                                   lowZ, highZ,
                                                   option), id);
}

/// Book 2-D profile.  Option is one of: " ", "s" (default), "i", "G"
/// (see TProfile2D::BuildOptions).  The number of channels in Z is
/// disregarded in a 2-D profile.
MonitorElement* DQMStore::bookProfile2D(char_string const& name,
                                        char_string const& title, int nchX,
                                        double lowX, double highX, int nchY,
                                        double lowY, double highY, double lowZ,
                                        double highZ,
                                        char const* option /* = "s" */, MEIdentifier const& id)
{
  return bookProfile2D_(pwd_, name, new TProfile2D(name, title,
                                                   nchX, lowX, highX,
                                                   nchY, lowY, highY,
                                                   lowZ, highZ,
                                                   option), id);
}

/// Book TProfile2D by cloning an existing profile.
MonitorElement* DQMStore::bookProfile2D(char_string const& name,
                                        TProfile2D* source, MEIdentifier const& id)
{
    return bookProfile2D_(pwd_, name, static_cast<TProfile2D *>(source->Clone(name)), id);
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
bool DQMStore::checkBinningMatches(MonitorElement* me, TH1* h, unsigned verbose)
{
  if (me->getTH1()->GetNbinsX() != h->GetNbinsX() ||
      me->getTH1()->GetNbinsY() != h->GetNbinsY() ||
      me->getTH1()->GetNbinsZ() != h->GetNbinsZ() ||
      me->getTH1()->GetXaxis()->GetXmin() != h->GetXaxis()->GetXmin() ||
      me->getTH1()->GetYaxis()->GetXmin() != h->GetYaxis()->GetXmin() ||
      me->getTH1()->GetZaxis()->GetXmin() != h->GetZaxis()->GetXmin() ||
      me->getTH1()->GetXaxis()->GetXmax() != h->GetXaxis()->GetXmax() ||
      me->getTH1()->GetYaxis()->GetXmax() != h->GetYaxis()->GetXmax() ||
      me->getTH1()->GetZaxis()->GetXmax() != h->GetZaxis()->GetXmax() ||
      !MonitorElement::CheckBinLabels((TAxis*)me->getTH1()->GetXaxis(),
                                      (TAxis*)h->GetXaxis()) ||
      !MonitorElement::CheckBinLabels((TAxis*)me->getTH1()->GetYaxis(),
                                      (TAxis*)h->GetYaxis()) ||
      !MonitorElement::CheckBinLabels((TAxis*)me->getTH1()->GetZaxis(),
                                      (TAxis*)h->GetZaxis())) {
    if (verbose > 0)
      std::cout
          << "*** DQMStore: WARNING:"
          << "checkBinningMatches: different binning - cannot add object '"
          << h->GetName() << "' of type " << h->IsA()->GetName()
          << " to existing ME: '" << me->getFullname() << "'\n";
    return false;
  }
  return true;
}

void DQMStore::collate1D(MonitorElement* me, TH1F* h, unsigned verbose)
{
  if (checkBinningMatches(me, h, verbose))
    me->getTH1F()->Add(h);
}

void DQMStore::collate1S(MonitorElement* me, TH1S* h, unsigned verbose)
{
  if (checkBinningMatches(me, h, verbose))
    me->getTH1S()->Add(h);
}

void DQMStore::collate1DD(MonitorElement* me, TH1D* h, unsigned verbose)
{
  if (checkBinningMatches(me, h, verbose))
    me->getTH1D()->Add(h);
}

void DQMStore::collate2D(MonitorElement* me, TH2F* h, unsigned verbose)
{
  if (checkBinningMatches(me, h, verbose))
    me->getTH2F()->Add(h);
}

void DQMStore::collate2S(MonitorElement* me, TH2S* h, unsigned verbose)
{
  if (checkBinningMatches(me, h, verbose))
    me->getTH2S()->Add(h);
}

void DQMStore::collate2DD(MonitorElement* me, TH2D* h, unsigned verbose)
{
  if (checkBinningMatches(me, h, verbose))
    me->getTH2D()->Add(h);
}

void DQMStore::collate3D(MonitorElement* me, TH3F* h, unsigned verbose)
{
  if (checkBinningMatches(me, h, verbose))
    me->getTH3F()->Add(h);
}

void DQMStore::collateProfile(MonitorElement* me, TProfile* h, unsigned verbose)
{
  if (checkBinningMatches(me, h, verbose)) {
    TProfile* meh = me->getTProfile();
    me->addProfiles(h, meh, meh, 1, 1);
  }
}

void DQMStore::collateProfile2D(MonitorElement* me, TProfile2D* h,
                                unsigned verbose)
{
  if (checkBinningMatches(me, h, verbose)) {
    TProfile2D* meh = me->getTProfile2D();
    me->addProfiles(h, meh, meh, 1, 1);
  }
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/// tag ME as <myTag> (myTag > 0)
void DQMStore::tag(MonitorElement* me, unsigned int const myTag)
{
  if (!myTag)
    raiseDQMError("DQMStore", "Attempt to tag monitor element '%s'"
                              " with a zero tag",
                  me->getFullname().c_str());
  if ((me->data_.flags & DQMNet::DQM_PROP_TAGGED) && myTag != me->data_.tag)
    raiseDQMError("DQMStore", "Attempt to tag monitor element '%s'"
                              " twice with multiple tags",
                  me->getFullname().c_str());

  me->data_.tag = myTag;
  me->data_.flags |= DQMNet::DQM_PROP_TAGGED;
}

/// tag ME specified by full pathname (e.g. "my/long/dir/my_histo")
void DQMStore::tag(std::string const& path, unsigned int const myTag)
{
  std::string dir;
  std::string name;
  splitPath(dir, name, path);

  if (auto me = findObject(dir, name))
    tag(me, myTag);
  else
    raiseDQMError("DQMStore", "Attempt to tag non-existent monitor element"
                              " '%s' with tag %u",
                  path.c_str(), myTag);
}

/// tag all children of folder (does NOT include subfolders)
void DQMStore::tagContents(std::string const& path, unsigned int const myTag)
{
  MonitorElement const proto{&path, std::string()};
  auto e = data_.end();
  auto i = data_.lower_bound(proto);
  for (; i != e && path == *i->data_.dirname; ++i)
    tag(const_cast<MonitorElement*>(&*i), myTag);
}

/// tag all children of folder, including all subfolders and their children;
/// path must be an exact path name
void DQMStore::tagAllContents(std::string const& path, unsigned int const myTag)
{
  std::string clean;
  std::string const* cleaned{nullptr};
  cleanTrailingSlashes(path, clean, cleaned);
  MonitorElement proto(cleaned, std::string());

  // FIXME: WILDCARDS? Old one supported them, but nobody seemed to use them.
  auto e = data_.end();
  auto i = data_.lower_bound(proto);
  while (i != e && isSubdirectory(*cleaned, *i->data_.dirname)) {
    tag(const_cast<MonitorElement*>(&*i), myTag);
    ++i;
  }
}

//////////////////////////////////////////////////////////////////////
/// get list of subdirectories of current directory
std::vector<std::string> DQMStore::getSubdirs() const
{
  std::vector<std::string> result;
  auto e = dirs_.end();
  auto i = dirs_.find(pwd_);

  // If we didn't find current directory, the tree is empty, so quit.
  if (i == e)
    return result;

  // Skip the current directory and then start looking for immediate
  // subdirectories in the dirs_ list.  Stop when we are no longer in
  // (direct or indirect) subdirectories of pwd_.  Note that we don't
  // "know" which order the set will sort A/B, A/B/C and A/D.
  while (++i != e && isSubdirectory(pwd_, *i))
    if (i->find('/', pwd_.size() + 1) == std::string::npos)
      result.push_back(*i);

  return result;
}

/// get list of (non-dir) MEs of current directory
std::vector<std::string> DQMStore::getMEs() const
{
  MonitorElement const proto{&pwd_, std::string()};
  std::vector<std::string> result;
  auto e = data_.end();
  auto i = data_.lower_bound(proto);
  for ( ; i != e && isSubdirectory(pwd_, *i->data_.dirname); ++i)
    if (pwd_ == *i->data_.dirname)
      result.push_back(i->getName());

  return result;
}

/// true if directory (or any subfolder at any level below it) contains
/// at least one monitorable element
bool DQMStore::containsAnyMonitorable(std::string const& path) const
{
  MonitorElement const proto{&path, std::string()};
  auto e = data_.end();
  auto i = data_.lower_bound(proto);
  return (i != e && isSubdirectory(path, *i->data_.dirname));
}

/// get ME from full pathname (e.g. "my/long/dir/my_histo")
MonitorElement* DQMStore::get(std::string const& path) const
{
  std::string dir;
  std::string name;
  splitPath(dir, name, path);
  MonitorElement const proto{&dir, name};
  auto mepos = data_.find(proto);
  return (mepos == data_.end() ? nullptr
                               : const_cast<MonitorElement*>(&*mepos));
}

/// get all MonitorElements tagged as <tag>
std::vector<MonitorElement*> DQMStore::get(unsigned int tag) const
{
  // FIXME: Use reverse map [tag -> path] / [tag -> dir]?
  std::vector<MonitorElement*> result;
  for (auto const& me : data_) {
    if ((me.data_.flags & DQMNet::DQM_PROP_TAGGED) && me.data_.tag == tag)
      result.push_back(const_cast<MonitorElement*>(&me));
  }
  return result;
}

/// get vector with all children of folder
/// (does NOT include contents of subfolders)
std::vector<MonitorElement*>
DQMStore::getContents(std::string const& path) const
{
  std::string clean;
  std::string const* cleaned{nullptr};
  cleanTrailingSlashes(path, clean, cleaned);
  MonitorElement proto(cleaned, std::string());

  std::vector<MonitorElement*> result;
  auto e = data_.end();
  auto i = data_.lower_bound(proto);
  for (; i != e && isSubdirectory(*cleaned, *i->data_.dirname); ++i)
    if (*cleaned == *i->data_.dirname)
      result.push_back(const_cast<MonitorElement*>(&*i));

  return result;
}

/// same as above for tagged MonitorElements
std::vector<MonitorElement*> DQMStore::getContents(std::string const& path,
                                                   unsigned int tag) const
{
  std::string clean;
  std::string const* cleaned{nullptr};
  cleanTrailingSlashes(path, clean, cleaned);
  MonitorElement proto(cleaned, std::string());

  std::vector<MonitorElement*> result;
  auto e = data_.end();
  auto i = data_.lower_bound(proto);
  for (; i != e && isSubdirectory(*cleaned, *i->data_.dirname); ++i)
    if (*cleaned == *i->data_.dirname &&
        (i->data_.flags & DQMNet::DQM_PROP_TAGGED) && i->data_.tag == tag)
      result.push_back(const_cast<MonitorElement*>(&*i));

  return result;
}

/// get contents;
/// return vector<string> of the form <dir pathname>:<obj1>,<obj2>,<obj3>;
/// if showContents = false, change form to <dir pathname>:
/// (useful for subscription requests; meant to imply "all contents")
void DQMStore::getContents(std::vector<std::string>& into,
                           bool showContents /* = true */) const
{
  into.clear();
  into.reserve(dirs_.size());

  auto me = data_.end();
  for (auto const& dir : dirs_) {
    if (!showContents) {
      into.emplace_back(dir+':');
      continue;
    }

    MonitorElement proto(&dir, std::string());
    auto mi = data_.lower_bound(proto);
    auto m = mi;
    size_t sz = dir.size() + 2;
    size_t nfound = 0;
    for (; m != me && isSubdirectory(dir, *m->data_.dirname); ++m) {
      if (dir == *m->data_.dirname) {
        sz += m->data_.objname.size() + 1;
        ++nfound;
      }
    }

    if (!nfound)
      continue;

    std::string str;
    str.reserve(sz);
    str += dir;
    str += ':';

    for (sz = 0; mi != m; ++mi) {
      if (dir != *mi->data_.dirname)
        continue;

      if (sz > 0)
        str += ',';

      str += mi->data_.objname;
      ++sz;
    }
    into.push_back(std::move(str));
  }
}

/// get MonitorElement <name> in directory <dir>
/// (null if MonitorElement does not exist)
MonitorElement* DQMStore::findObject(std::string const& dir,
                                     std::string const& name,
                                     MEIdentifier const& id) const
{
  if (dir.find_first_not_of(s_safe) != std::string::npos)
    raiseDQMError("DQMStore", "Monitor element path name '%s' uses"
                              " unacceptable characters",
                  dir.c_str());
  if (name.find_first_not_of(s_safe) != std::string::npos)
    raiseDQMError("DQMStore", "Monitor element path name '%s' uses"
                              " unacceptable characters",
                  name.c_str());

  // auto meit = safe_data_.find(KeyType{id.streamId, id.moduleId});
  // if (meit != safe_data_.cend()) {
  //   return nullptr;
  // }

  MonitorElement proto;
  proto.data_.dirname = &dir;
  proto.data_.objname = name;
  proto.data_.run = id.run;
  proto.data_.lumi = id.lumi;
  proto.data_.streamId = id.streamId;
  proto.data_.moduleId = id.moduleId;

  // auto const& mes = meit->second;
  auto const& mes = data_;
  auto mepos = mes.find(proto);
  return (mepos == mes.end() ? nullptr : const_cast<MonitorElement*>(&*mepos));
}

/** get tags for various maps, return vector with strings of the form
    <dir pathname>:<obj1>/<tag1>/<tag2>,<obj2>/<tag1>/<tag3>, etc. */
void DQMStore::getAllTags(std::vector<std::string>& into) const
{
  into.clear();
  into.reserve(dirs_.size());

  auto me = data_.end();
  char tagbuf[32]; // more than enough for '/' and up to 10 digits

  for (auto const& dir : dirs_) {
    MonitorElement proto(&dir, std::string());
    auto mi = data_.lower_bound(proto);
    auto m = mi;
    size_t sz = dir.size() + 2;
    size_t nfound = 0;
    for (; m != me && isSubdirectory(dir, *m->data_.dirname); ++m)
      if (dir == *m->data_.dirname &&
          (m->data_.flags & DQMNet::DQM_PROP_TAGGED)) {
        // the tags count for '/' + up to 10 digits, otherwise ',' + ME name
        sz += 1 + m->data_.objname.size() + 11;
        ++nfound;
      }

    if (!nfound)
      continue;

    auto& str = *into.insert(into.end(), std::string());
    str.reserve(sz);

    str += dir;
    str += ':';
    for (sz = 0; mi != m; ++mi) {
      if (dir == *m->data_.dirname &&
          (m->data_.flags & DQMNet::DQM_PROP_TAGGED)) {
        sprintf(tagbuf, "/%u", mi->data_.tag);
        if (sz > 0)
          str += ',';
        str += m->data_.objname;
        str += tagbuf;
        ++sz;
      }
    }
  }
}

/// get vector with children of folder, including all subfolders + their
/// children;
/// must use an exact pathname
std::vector<MonitorElement*>
DQMStore::getAllContents(std::string const& path, uint32_t runNumber /* = 0 */,
                         uint32_t lumi /* = 0 */) const
{
  std::string clean;
  std::string const* cleaned{nullptr};
  cleanTrailingSlashes(path, clean, cleaned);
  MonitorElement proto(cleaned, std::string(), runNumber);
  proto.setLumi(lumi);

  std::vector<MonitorElement*> result;
  auto e = data_.end();
  auto i = data_.lower_bound(proto);
  for (; i != e && isSubdirectory(*cleaned, *i->data_.dirname); ++i) {
    if (runNumber != 0) {
      if (i->data_.run > runNumber // TODO[rovere]: pleonastic? first we
                                   // encounter local ME of the same run ...
          || i->data_.streamId != 0 || i->data_.moduleId != 0)
        break;
    }
    if (lumi != 0) {
      if (i->data_.lumi > lumi || i->data_.streamId != 0 ||
          i->data_.moduleId != 0)
        break;
    }
    if (runNumber != 0 or lumi != 0) {
      assert(i->data_.streamId == 0);
      assert(i->data_.moduleId == 0);
    }
    result.push_back(const_cast<MonitorElement*>(&*i));
  }

  if (enableMultiThread_) {
    // save legacy modules when running MT
    i = data_.begin();
    for (; i != e && isSubdirectory(*cleaned, *i->data_.dirname); ++i) {
      if (i->data_.run != 0 || i->data_.streamId != 0 || i->data_.moduleId != 0)
        break;
      result.push_back(const_cast<MonitorElement*>(&*i));
    }
  }

  return result;
}

/// get vector with children of folder, including all subfolders + their
/// children;
/// matches names against a wildcard pattern matched against the full ME path
std::vector<MonitorElement*> DQMStore::getMatchingContents(
    std::string const& pattern,
    lat::Regexp::Syntax syntaxType /* = Wildcard */) const
{
  lat::Regexp rx;
  try {
    rx = lat::Regexp(pattern, 0, syntaxType);
    rx.study();
  }
  catch (lat::Error& e) {
    raiseDQMError("DQMStore", "Invalid regular expression '%s': %s",
                  pattern.c_str(), e.explain().c_str());
  }

  std::string path;
  std::vector<MonitorElement*> result;
  auto i = data_.begin();
  auto e = data_.end();
  for (; i != e; ++i)
    {
      path.clear();
      mergePath(path, *i->data_.dirname, i->data_.objname);
      if (rx.match(path))
        result.push_back(const_cast<MonitorElement*>(&*i));
    }

  return result;
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/** Invoke this method after flushing all recently changed monitoring.
    Clears updated flag on all recently updated MEs and calls their
    Reset() method for those that have resetMe = true. */
void DQMStore::reset()
{
  for (auto const& me : data_) {
    if (!me.wasUpdated())
      continue;

    auto& nonConstMe = const_cast<MonitorElement&>(me);
    if (nonConstMe.resetMe())
      nonConstMe.Reset();
    nonConstMe.resetUpdate();
  }

  reset_ = true;
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/** Invoke this method after flushing all recently changed monitoring.
    Clears updated flag on all MEs and calls their Reset() method. */
void DQMStore::forceReset()
{
  for (auto const& me : data_) {
    if (forceResetOnBeginLumi_ && (me.getLumiFlag() == false))
      continue;
    auto& nonConstMe = const_cast<MonitorElement&>(me);
    nonConstMe.Reset();
    nonConstMe.resetUpdate();
  }

  reset_ = true;
}


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/** Called after all globalBeginLuminosityBlock.
 * Reset global per-lumi MEs (or all MEs if LSbasedMode) so that
 * they can be reused.
 */
void
DQMStore::postGlobalBeginLumi(const edm::GlobalContext &gc)
{
  static const std::string null_str("");

  auto const& lumiblock = gc.luminosityBlockID();
  uint32_t run = lumiblock.run();

  // find the range of non-legacy global MEs for the current run:
  // run != 0, lumi == 0 (implicit), stream id == 0, module id == 0
  const MonitorElement begin(&null_str, null_str, run, 0, 0);
  const MonitorElement end(&null_str, null_str, run, 0, 1);
  auto i = data_.lower_bound(begin);
  const auto e = data_.lower_bound(end);
  while (i != e) {
    auto& me = const_cast<MonitorElement&>(*i++);
    // skip per-run MEs
    if (not LSbasedMode_ and not me.getLumiFlag())
      continue;
    me.Reset();
    me.resetUpdate();
  }
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/** Delete *global* histograms which are no longer in use.
 * Such histograms are created at the end of each lumi and should be
 * deleted after the last globalEndLuminosityBlock.
 */
void DQMStore::deleteUnusedLumiHistograms(uint32_t run, uint32_t lumi)
{
  if (!enableMultiThread_)
    return;

  std::lock_guard<std::mutex> guard(book_mutex_);

  MonitorElement proto{&s_empty_string, s_empty_string, run, 0, 0};
  proto.setLumi(lumi);

  auto e = data_.end();
  auto i = data_.lower_bound(proto);

  while (i != e) {
    if (i->data_.streamId != 0 || i->data_.moduleId != 0)
      break;
    if (i->data_.lumi != lumi)
      break;
    if (i->data_.run != run)
      break;
    if (not i->markedToDelete()) {
      ++i;
      continue;
    }

    if (verbose_ > 1) {
      std::cout
          << "DQMStore::deleteUnusedLumiHistograms: deleted monitor element '"
          << *i->data_.dirname << "/" << i->data_.objname << "'"
          << "flags " << i->data_.flags << "\n";
    }

    i = data_.erase(i);
  }
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/// extract object (TH1F, TH2F, ...) from <to>; return success flag
/// flag fromRemoteNode indicating if ME arrived from different node
bool DQMStore::extract(TObject* obj, std::string const& dir,
                       bool const overwrite, bool const collateHistograms)
{
  // NB: Profile histograms inherit from TH*D, checking order matters.
  MonitorElement *refcheck = nullptr;
  if (auto *h = dynamic_cast<TProfile *>(obj))
  {
    MonitorElement *me = findObject(dir, h->GetName());
    if (! me)
      me = bookProfile_(dir, h->GetName(), (TProfile *) h->Clone(), MEIdentifier{});
    else if (overwrite)
      me->copyFrom(h);
    else if (isCollateME(me) || collateHistograms)
      collateProfile(me, h, verbose_);
    refcheck = me;
  }
  else if (auto *h = dynamic_cast<TProfile2D *>(obj))
  {
    MonitorElement *me = findObject(dir, h->GetName());
    if (! me)
      me = bookProfile2D_(dir, h->GetName(), (TProfile2D *) h->Clone(), MEIdentifier{});
    else if (overwrite)
      me->copyFrom(h);
    else if (isCollateME(me) || collateHistograms)
      collateProfile2D(me, h, verbose_);
    refcheck = me;
  }
  else if (auto *h = dynamic_cast<TH1F *>(obj))
  {
    MonitorElement *me = findObject(dir, h->GetName());
    if (! me)
      me = book1D_(dir, h->GetName(), (TH1F *) h->Clone(), MEIdentifier{});
    else if (overwrite)
      me->copyFrom(h);
    else if (isCollateME(me) || collateHistograms)
      collate1D(me, h, verbose_);
    refcheck = me;
  }
  else if (auto *h = dynamic_cast<TH1S *>(obj))
  {
    MonitorElement *me = findObject(dir, h->GetName());
    if (! me)
      me = book1S_(dir, h->GetName(), (TH1S *) h->Clone(), MEIdentifier{});
    else if (overwrite)
      me->copyFrom(h);
    else if (isCollateME(me) || collateHistograms)
      collate1S(me, h, verbose_);
    refcheck = me;
  }
  else if (auto *h = dynamic_cast<TH1D *>(obj))
  {
    MonitorElement *me = findObject(dir, h->GetName());
    if (! me)
      me = book1DD_(dir, h->GetName(), (TH1D *) h->Clone(), MEIdentifier{});
    else if (overwrite)
      me->copyFrom(h);
    else if (isCollateME(me) || collateHistograms)
      collate1DD(me, h, verbose_);
    refcheck = me;
  }
  else if (auto *h = dynamic_cast<TH2F *>(obj))
  {
    MonitorElement *me = findObject(dir, h->GetName());
    if (! me)
      me = book2D_(dir, h->GetName(), (TH2F *) h->Clone(), MEIdentifier{});
    else if (overwrite)
      me->copyFrom(h);
    else if (isCollateME(me) || collateHistograms)
      collate2D(me, h, verbose_);
    refcheck = me;
  }
  else if (auto *h = dynamic_cast<TH2S *>(obj))
  {
    MonitorElement *me = findObject(dir, h->GetName());
    if (! me)
      me = book2S_(dir, h->GetName(), (TH2S *) h->Clone(), MEIdentifier{});
    else if (overwrite)
      me->copyFrom(h);
    else if (isCollateME(me) || collateHistograms)
      collate2S(me, h, verbose_);
    refcheck = me;
  }
  else if (auto *h = dynamic_cast<TH2D *>(obj))
  {
    MonitorElement *me = findObject(dir, h->GetName());
    if (! me)
      me = book2DD_(dir, h->GetName(), (TH2D *) h->Clone(), MEIdentifier{});
    else if (overwrite)
      me->copyFrom(h);
    else if (isCollateME(me) || collateHistograms)
      collate2DD(me, h, verbose_);
    refcheck = me;
  }
  else if (auto *h = dynamic_cast<TH3F *>(obj))
  {
    MonitorElement *me = findObject(dir, h->GetName());
    if (! me)
      me = book3D_(dir, h->GetName(), (TH3F *) h->Clone(), MEIdentifier{});
    else if (overwrite)
      me->copyFrom(h);
    else if (isCollateME(me) || collateHistograms)
      collate3D(me, h, verbose_);
    refcheck = me;
  }
  else if (dynamic_cast<TObjString *>(obj))
  {
    lat::RegexpMatch m;
    if (! s_rxmeval.match(obj->GetName(), 0, 0, &m))
      {
        if (strstr(obj->GetName(), "CMSSW"))
          {
            if (verbose_)
              std::cout << "Input file version: " << obj->GetName() << std::endl;
            return true;
          }
        else if (strstr(obj->GetName(), "DQMPATCH"))
          {
            if (verbose_)
              std::cout << "DQM patch version: " << obj->GetName() << std::endl;
            return true;
          }
        else
          {
            std::cout << "*** DQMStore: WARNING: cannot extract object '"
                      << obj->GetName() << "' of type '"
                      << obj->IsA()->GetName() << "'\n";
            return false;
          }
      }

      std::string label = m.matchString(obj->GetName(), 1);
      std::string kind = m.matchString(obj->GetName(), 2);
      std::string value = m.matchString(obj->GetName(), 3);

      if (kind == "i")
        {
          MonitorElement *me = findObject(dir, label);
          if (! me || overwrite)
            {
              if (! me) me = bookInt_(dir, label, MEIdentifier{});
              me->Fill(atoll(value.c_str()));
            }
        }
      else if (kind == "f")
        {
          MonitorElement *me = findObject(dir, label);
          if (! me || overwrite)
            {
              if (! me) me = bookFloat_(dir, label, MEIdentifier{});
              me->Fill(atof(value.c_str()));
            }
        }
      else if (kind == "s")
        {
          MonitorElement *me = findObject(dir, label);
          if (! me)
            me = bookString_(dir, label, value, MEIdentifier{});
          else if (overwrite)
            me->Fill(value);
        }
      else if (kind == "e")
        {
          MonitorElement* me = findObject(dir, label);
          if (!me)
            {
              std::cout << "*** DQMStore: WARNING: no monitor element '"
                        << label << "' in directory '"
                        << dir << "' to be marked as efficiency plot.\n";
              return false;
            }
          me->setEfficiencyFlag();
        }
      else if (kind == "t")
        {
          MonitorElement* me = findObject(dir, label);
          if (!me)
            {
              std::cout << "*** DQMStore: WARNING: no monitor element '"
                        << label << "' in directory '"
                        << dir << "' for a tag\n";
              return false;
            }
          errno = 0;
          char* endp = nullptr;
          unsigned long val = strtoul(value.c_str(), &endp, 10);
          if ((val == 0 && errno) || *endp || val > ~uint32_t(0))
            {
              std::cout << "*** DQMStore: WARNING: cannot restore tag '"
                        << value << "' for monitor element '"
                        << label << "' in directory '"
                        << dir << "' - invalid value\n";
              return false;
            }
          tag(me, val);
        }
      else if (kind == "qr")
        {
          // Handle qreports, but skip them while reading in references.
          if (!isSubdirectory(s_referenceDirName, dir))
            {
              size_t dot = label.find('.');
              if (dot == std::string::npos)
                {
                  std::cout << "*** DQMStore: WARNING: quality report label in '" << label
                            << "' is missing a '.' and cannot be extracted\n";
                  return false;
                }

              std::string mename (label, 0, dot);
              std::string qrname (label, dot+1, std::string::npos);

              m.reset();
              DQMNet::QValue qv;
              if (s_rxmeqr1.match(value, 0, 0, &m))
                {
                  qv.code = atoi(m.matchString(value, 1).c_str());
                  qv.qtresult = strtod(m.matchString(value, 2).c_str(), 0);
                  qv.message = m.matchString(value, 4);
                  qv.qtname = qrname;
                  qv.algorithm = m.matchString(value, 3);
                }
              else if (s_rxmeqr2.match(value, 0, 0, &m))
                {
                  qv.code = atoi(m.matchString(value, 1).c_str());
                  qv.qtresult = 0; // unavailable in old format
                  qv.message = m.matchString(value, 2);
                  qv.qtname = qrname;
                  // qv.algorithm unavailable in old format
                }
              else
                {
                  std::cout << "*** DQMStore: WARNING: quality test value '"
                            << value << "' is incorrectly formatted\n";
                  return false;
                }

              MonitorElement* me = findObject(dir, mename);
              if (!me)
                {
                  std::cout << "*** DQMStore: WARNING: no monitor element '"
                            << mename << "' in directory '"
                            << dir << "' for quality test '"
                            << label << "'\n";
                  return false;
                }

              me->addQReport(qv, /* FIXME: getQTest(qv.qtname)? */ 0);
            }
        }
  }
  else if (auto *n = dynamic_cast<TNamed *>(obj))
    {
      // For old DQM data.
      std::string s;
    s.reserve(6 + strlen(n->GetTitle()) + 2*strlen(n->GetName()));
    s += '<'; s += n->GetName(); s += '>';
    s += n->GetTitle();
    s += '<'; s += '/'; s += n->GetName(); s += '>';
    TObjString os(s.c_str());
    return extract(&os, dir, overwrite, collateHistograms_);
    }
  else
  {
    std::cout << "*** DQMStore: WARNING: cannot extract object '"
              << obj->GetName() << "' of type '" << obj->IsA()->GetName()
              << "' and with title '" << obj->GetTitle() << "'\n";
    return false;
  }

  // If we just read in a reference MonitorElement, and there is a
  // MonitorElement with the same name, link the two together.
  // The other direction is handled by the book() method.
  if (refcheck && isSubdirectory(s_referenceDirName, dir)) {
    std::string mdir(dir, s_referenceDirName.size() + 1, std::string::npos);
    if (auto master = findObject(mdir, obj->GetName())) {
      // We have extracted a MonitorElement, and it's located in the reference
      // dir. Then we find the corresponding MonitorElement in the
      // non-reference dir and assign the object_ of the reference
      // MonitorElement to the reference_ property of the corresponding
      // non-reference MonitorElement.
      master->data_.flags |= DQMNet::DQM_PROP_HAS_REFERENCE;
      master->reference_ = refcheck->object_;
    }
  }

  return true;
}

/// Use this for saving monitoring objects in ROOT files with dir structure;
/// cd into directory (create first if it doesn't exist);
/// returns success flag
bool DQMStore::cdInto(std::string const& path) const
{
  assert(!path.empty());

  // Find the first path component.
  size_t start = 0;
  size_t end = path.find('/', start);
  if (end == std::string::npos)
    end = path.size();

  while (true) {
    // Check if this subdirectory component exists.  If yes, make sure
    // it is actually a subdirectory.  Otherwise create or cd into it.
    std::string part(path, start, end - start);
    auto* o = gDirectory->Get(part.c_str());
    if (o && !dynamic_cast<TDirectory*>(o))
      raiseDQMError("DQMStore",
                    "Attempt to create directory '%s' in a file"
                    " fails because the part '%s' already exists and is not"
                    " directory",
                    path.c_str(), part.c_str());
    else if (!o)
      gDirectory->mkdir(part.c_str());

    if (!gDirectory->cd(part.c_str()))
      raiseDQMError("DQMStore",
                    "Attempt to create directory '%s' in a file"
                    " fails because could not cd into subdirectory '%s'",
                    path.c_str(), part.c_str());

    // Stop if we reached the end, ignoring any trailing '/'.
    if (end + 1 >= path.size())
      break;

    // Find the next path component.
    start = end + 1;
    end = path.find('/', start);
    if (end == std::string::npos)
      end = path.size();
  }

  return true;
}

void
DQMStore::saveMonitorElementToROOT(
    MonitorElement const& me,
    TFile & file)
{
  // Save the object.
  if (me.kind() < MonitorElement::DQM_KIND_TH1F) {
    TObjString(me.tagString().c_str()).Write();
  } else {
    me.object_->Write();
  }

  // Save quality reports if this is not in reference section.
  if (not isSubdirectory(s_referenceDirName, *me.data_.dirname))
  {
    for (auto const& report: me.data_.qreports) {
      TObjString(me.qualityTagString(report).c_str()).Write();
    }
  }

  // Save efficiency tag, if any.
  if (me.data_.flags & DQMNet::DQM_PROP_EFFICIENCY_PLOT) {
    TObjString(me.effLabelString().c_str()).Write();
  }

  // Save tag if any.
  if (me.data_.flags & DQMNet::DQM_PROP_TAGGED) {
    TObjString(me.tagLabelString().c_str()).Write();
  }
}

void
DQMStore::saveMonitorElementRangeToROOT(
    std::string const& dir,
    std::string const& refpath,
    SaveReferenceTag ref,
    int minStatus,
    unsigned int run,
    MEMap::const_iterator begin,
    MEMap::const_iterator end,
    TFile & file,
    unsigned int & counter)
{
  for (auto const& me: boost::make_iterator_range(begin, end))
  {
    if (not isSubdirectory(dir, *me.data_.dirname))
      break;

    if (verbose_ > 1)
      std::cout << "DQMStore::save:"
                << " run: " << me.run()
                << " lumi: " << me.lumi()
                << " lumiFlag: " << me.getLumiFlag()
                << " streamId: " << me.streamId()
                << " moduleId: " << me.moduleId()
                << " fullpathname: " << me.getFullname()
                << " flags: " << std::hex << me.data_.flags
                << std::endl;

    // Skip MonitorElements in a subdirectory of the current one.
    if (dir != *me.data_.dirname) {
      if (verbose_ > 1) {
        std::cout << "DQMStore::save: skipping monitor element in a subfolder of " << dir << "/" << std::endl;
      }
      continue;
    }

    // For MonitorElements booked with the thread-safe approach, identified
    // by having run != 0, ignore the per-stream ones.
    if (run != 0 and (me.data_.streamId != 0 or me.data_.moduleId != 0)) {
      if (verbose_ > 1) {
        std::cout << "DQMStore::save: skipping per-stream monitor element" << std::endl;
      }
      continue;
    }

    // Handle reference histograms, with three distinct cases:
    // 1) Skip all references entirely on saving.
    // 2) Blanket saving of all references.
    // 3) Save only references for monitor elements with qtests.
    // The latter two are affected by "path" sub-tree selection,
    // i.e. references are saved only in the selected tree part.
    if (isSubdirectory(refpath, *me.data_.dirname))
    {
      if (ref == SaveWithoutReference)
        // Skip the reference entirely.
        continue;
      else if (ref == SaveWithReference)
        // Save all references regardless of qtests.
        ;
      else if (ref == SaveWithReferenceForQTest)
      {
        // Save only references for monitor elements with qtests
        // with an optional cut on minimum quality test result.
        int status = -1;
        std::string mname(me.getFullname(), s_referenceDirName.size()+1, std::string::npos);
        MonitorElement *master = get(mname);
        if (master)
          for (size_t i = 0, e = master->data_.qreports.size(); i != e; ++i)
            status = std::max(status, master->data_.qreports[i].code);

        if (not master or status < minStatus)
        {
          if (verbose_ > 1)
            std::cout << "DQMStore::save: skipping monitor element '"
                      << me.data_.objname << "' while saving, status is "
                      << status << ", required minimum status is "
                      << minStatus << std::endl;
          continue;
        }
      }
    }

    if (verbose_ > 1) {
      std::cout << "DQMStore::save: saving monitor element" << std::endl;
    }

    saveMonitorElementToROOT(me, file);

    // Count saved histograms
    ++counter;
  }
}

/// save directory with monitoring objects into protobuf file <filename>;
/// if directory="", save full monitoring structure
void
DQMStore::save(const std::string &filename,
               const std::string &path /* = "" */,
               const std::string &pattern /* = "" */,
               const std::string &rewrite /* = "" */,
               const uint32_t run /* = 0 */,
               const uint32_t lumi /* = 0 */,
               SaveReferenceTag ref /* = SaveWithReference */,
               int minStatus /* = dqm::qstatus::STATUS_OK */,
               const std::string &fileupdate /* = RECREATE */)
{
  // TFile flushes to disk with fsync() on every TDirectory written to
  // the file.  This makes DQM file saving painfully slow, and
  // ironically makes it _more_ likely the file saving gets
  // interrupted and corrupts the file.  The utility class below
  // simply ignores the flush synchronisation.
  class TFileNoSync : public TFile {
  public:
    TFileNoSync(const char *file, const char *opt) : TFile(file, opt) {}
    Int_t SysSync(Int_t) override { return 0; }
  };

  std::lock_guard<std::mutex> guard(book_mutex_);

  unsigned int nme = 0;

  // open output file, on 1st save recreate, later update
  if (verbose_) {
    std::cout << "DQMStore::save: Opening TFile '" << filename
              << "' with option '" << fileupdate << "'"
              << std::endl;
  }

  TFileNoSync f{filename.c_str(), fileupdate.c_str()}; // open file
  if (f.IsZombie())
    raiseDQMError("DQMStore", "Failed to create/update file '%s'",
                  filename.c_str());
  f.cd();

  // Construct a regular expression from the pattern string.
  std::unique_ptr<lat::Regexp> rxpat;
  if (!pattern.empty())
    rxpat = std::make_unique<lat::Regexp>(pattern);

  // Prepare a path for the reference object selection.
  std::string refpath;
  refpath.reserve(s_referenceDirName.size() + path.size() + 2);
  refpath += s_referenceDirName;
  if (not path.empty())
  {
    refpath += '/';
    refpath += path;
  }

  // Loop over the directory structure.
  for (auto const& dir: dirs_)
  {
    // Check if we should process this directory.  We process the
    // requested part of the object tree, including references.
    if (not path.empty()
        and not isSubdirectory(refpath, dir)
        and not isSubdirectory(path, dir))
      continue;

    if (verbose_ > 1) {
      std::cout << "DQMStore::save: DQM folder " << dir << "/" << std::endl;
    }

    // Create the directory.
    gDirectory->cd("/");
    if (dir.empty())
      cdInto(s_monitorDirName);
    else if (rxpat.get())
      cdInto(s_monitorDirName + '/' + lat::StringOps::replace(dir, *rxpat, rewrite));
    else
      cdInto(s_monitorDirName + '/' + dir);

    // Loop over monitor elements in this directory.
    if (not enableMultiThread_) {
      MonitorElement proto(&dir, std::string(), run, 0, 0);
      auto begin = data_.lower_bound(proto);
      auto end   = data_.end();
      saveMonitorElementRangeToROOT(dir, refpath, ref, minStatus, run, begin, end, f, nme);
    } else {
      // Restrict the loop to the monitor elements for the current lumisection
      MonitorElement proto(&dir, std::string(), run, 0, 0);
      proto.setLumi(lumi);
      auto begin = data_.lower_bound(proto);
      proto.setLumi(lumi+1);
      auto end   = data_.lower_bound(proto);
      saveMonitorElementRangeToROOT(dir, refpath, ref, minStatus, run, begin, end, f, nme);
    }

    // In LSbasedMode, loop also over the (run, 0, 0, 0) global histograms;
    // these could be the merged global histrograms of their per-stream
    // counterparts after the streamEndRun transition - but they are not
    // produced in LSbasedMode.
    if (enableMultiThread_ and LSbasedMode_ and lumi != 0) {
      auto begin = data_.lower_bound(MonitorElement(&dir, std::string(), run, 0, 0));
      auto end   = data_.lower_bound(MonitorElement(&dir, std::string(), run, 0, 1));
      saveMonitorElementRangeToROOT(dir, refpath, ref, minStatus, run, begin, end, f, nme);
    }
  }

  f.Close();

  // Maybe make some noise.
  if (verbose_) {
    std::cout << "DQMStore::save: successfully wrote " << nme
              << " objects from path '" << path << "/"
              << "' into DQM file '" << filename << "'\n";
  }
}

void
DQMStore::saveMonitorElementToPB(
    MonitorElement const& me,
    dqmstorepb::ROOTFilePB & file)
{
  // Save the object.
  TBufferFile buffer(TBufferFile::kWrite);
  if (me.kind() < MonitorElement::DQM_KIND_TH1F) {
    TObjString object(me.tagString().c_str());
    buffer.WriteObject(&object);
  } else {
    buffer.WriteObject(me.object_);
  }
  dqmstorepb::ROOTFilePB::Histo & histo = * file.add_histo();
  histo.set_full_pathname(*me.data_.dirname + '/' + me.data_.objname);
  histo.set_flags(me.data_.flags);
  histo.set_size(buffer.Length());
  histo.set_streamed_histo((const void*)buffer.Buffer(), buffer.Length());

  // Save quality reports if this is not in reference section.
  // XXX not supported by protobuf files.

  // Save efficiency tag, if any.
  // XXX not supported by protobuf files.

  // Save tag if any.
  // XXX not supported by protobuf files.
}

void
DQMStore::saveMonitorElementRangeToPB(
    std::string const& dir,
    unsigned int run,
    MEMap::const_iterator begin,
    MEMap::const_iterator end,
    dqmstorepb::ROOTFilePB & file,
    unsigned int & counter)
{
  for (auto const& me: boost::make_iterator_range(begin, end))
  {
    if (not isSubdirectory(dir, *me.data_.dirname))
      break;

    if (verbose_ > 1)
      std::cout << "DQMStore::savePB:"
                << " run: " << me.run()
                << " lumi: " << me.lumi()
                << " lumiFlag: " << me.getLumiFlag()
                << " streamId: " << me.streamId()
                << " moduleId: " << me.moduleId()
                << " fullpathname: " << me.getFullname()
                << " flags: " << std::hex << me.data_.flags
                << std::endl;

    // Skip MonitorElements in a subdirectory of the current one.
    if (dir != *me.data_.dirname) {
      if (verbose_ > 1) {
        std::cout << "DQMStore::savePB: skipping monitor element in a subfolder of " << dir << "/" << std::endl;
      }
      continue;
    }

    // For MonitorElements booked with the thread-safe approach, identified
    // by having run != 0, ignore the per-stream ones.
    if (run != 0 and (me.data_.streamId != 0 or me.data_.moduleId != 0)) {
      if (verbose_ > 1) {
        std::cout << "DQMStore::savePB: skipping per-stream monitor element" << std::endl;
      }
      continue;
    }

    // Handle reference histograms, with three distinct cases:
    // XXX not supported by protobuf files.

    if (verbose_ > 1) {
      std::cout << "DQMStore::savePB: saving monitor element" << std::endl;
    }

    saveMonitorElementToPB(me, file);

    // Count saved histograms
    ++counter;
  }
}

/// save directory with monitoring objects into protobuf file <filename>;
/// if directory="", save full monitoring structure
void
DQMStore::savePB(const std::string &filename,
                 const std::string &path /* = "" */,
                 const uint32_t run /* = 0 */,
                 const uint32_t lumi /* = 0 */)
{
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::io::GzipOutputStream;
  using google::protobuf::io::StringOutputStream;

  std::lock_guard<std::mutex> guard(book_mutex_);

  unsigned int nme = 0;

  if (verbose_) {
    std::cout << "DQMStore::savePB: Opening PBFile '" << filename << "'"
              << std::endl;
  }
  dqmstorepb::ROOTFilePB dqmstore_message;

  // Loop over the directory structure.
  for (auto const& dir: dirs_)
  {
    // Check if we should process this directory.  We process the
    // requested part of the object tree, including references.
    if (not path.empty()
        and not isSubdirectory(path, dir))
      continue;

    if (verbose_ > 1) {
      std::cout << "DQMStore::savePB: DQM folder " << dir << "/" << std::endl;
    }

    // Loop over monitor elements in this directory.
    if (not enableMultiThread_) {
      MonitorElement proto(&dir, std::string(), run, 0, 0);
      auto begin = data_.lower_bound(proto);
      auto end   = data_.end();
      saveMonitorElementRangeToPB(dir, run, begin, end, dqmstore_message, nme);
    } else {
      // Restrict the loop to the monitor elements for the current lumisection
      MonitorElement proto(&dir, std::string(), run, 0, 0);
      proto.setLumi(lumi);
      auto begin = data_.lower_bound(proto);
      proto.setLumi(lumi+1);
      auto end   = data_.lower_bound(proto);
      saveMonitorElementRangeToPB(dir, run, begin, end, dqmstore_message, nme);
    }

    // In LSbasedMode, loop also over the (run, 0, 0, 0) global histograms;
    // these could be the merged global histrograms of their per-stream
    // counterparts after the streamEndRun transition - but they are not
    // produced in LSbasedMode.
    if (enableMultiThread_ and LSbasedMode_ and lumi != 0) {
      auto begin = data_.lower_bound(MonitorElement(&dir, std::string(), run, 0, 0));
      auto end   = data_.lower_bound(MonitorElement(&dir, std::string(), run, 0, 1));
      saveMonitorElementRangeToPB(dir, run, begin, end, dqmstore_message, nme);
    }
  }

  int filedescriptor = ::open(filename.c_str(),
                              O_WRONLY | O_CREAT | O_TRUNC,
                              S_IRUSR | S_IWUSR |
                              S_IRGRP | S_IWGRP |
                              S_IROTH);
  FileOutputStream file_stream(filedescriptor);
  GzipOutputStream::Options options;
  options.format = GzipOutputStream::GZIP;
  options.compression_level = 1;
  GzipOutputStream gzip_stream(&file_stream, options);
  dqmstore_message.SerializeToZeroCopyStream(&gzip_stream);

  // Flush the internal streams before closing the fd.
  gzip_stream.Close();
  file_stream.Close();
  ::close(filedescriptor);

  // Maybe make some noise.
  if (verbose_) {
    std::cout << "DQMStore::savePB: successfully wrote " << nme
              << " objects from path '" << path << "/"
              << "' into DQM file '" << filename << "'\n";
  }
}


/// read ROOT objects from file <file> in directory <onlypath>;
/// return total # of ROOT objects read
unsigned int DQMStore::readDirectory(TFile* file, bool const overwrite,
                                     std::string const& onlypath,
                                     std::string const& prepend,
                                     std::string const& curdir,
                                     OpenRunDirs stripdirs) // PRIVATE
{
  unsigned int ntot{};
  unsigned int count{};

  if (!file->cd(curdir.c_str()))
    raiseDQMError("DQMStore", "Failed to process directory '%s' while"
                              " reading file '%s'",
                  curdir.c_str(), file->GetName());

  // Figure out current directory name, but strip out the top
  // directory into which we dump everything.
  std::string dirpart{curdir};
  if (dirpart.compare(0, s_monitorDirName.size(), s_monitorDirName) == 0) {
    if (dirpart.size() == s_monitorDirName.size())
      dirpart.clear();
    else if (dirpart[s_monitorDirName.size()] == '/')
      dirpart.erase(0, s_monitorDirName.size() + 1);
  }

  // See if we are going to skip this directory.
  bool const skip{!onlypath.empty() && !isSubdirectory(onlypath, dirpart)};

  if (prepend == s_collateDirName || prepend == s_referenceDirName ||
      stripdirs == StripRunDirs) {
    // Remove Run # and RunSummary dirs
    // first look for Run summary,
    // if that is found and erased, also erase Run dir
    size_t slash = dirpart.find('/');
    size_t pos = dirpart.find("/Run summary");
    if (slash != std::string::npos && pos != std::string::npos) {
      dirpart.erase(pos, 12);

      pos = dirpart.find("Run ");
      size_t length = dirpart.find('/', pos + 1) - pos + 1;
      if (pos != std::string::npos)
        dirpart.erase(pos, length);
    }
  }

  // If we are prepending, add it to the directory name,
  // and suppress reading of already existing reference histograms
  if (prepend == s_collateDirName || prepend == s_referenceDirName) {
    size_t slash = dirpart.find('/');
    // If we are reading reference, skip previous reference.
    if (slash == std::string::npos // skip if Reference is toplevel folder, i.e.
                                   // no slash
        && slash + 1 + s_referenceDirName.size() == dirpart.size() &&
        dirpart.compare(slash + 1, s_referenceDirName.size(),
                        s_referenceDirName) == 0)
      return 0;

    slash = dirpart.find('/');
    // Skip reading of EventInfo subdirectory.
    if (slash != std::string::npos && slash + 10 == dirpart.size() &&
        dirpart.compare(slash + 1, 9, "EventInfo") == 0) {
      if (verbose_)
        std::cout << "DQMStore::readDirectory: skipping '" << dirpart << "'\n";
      return 0;
    }

    // Add prefix.
    if (dirpart.empty())
      dirpart = prepend;
    else
      dirpart = prepend + '/' + dirpart;
  }
  else if (!prepend.empty()) {
    if (dirpart.empty())
      dirpart = prepend;
    else
      dirpart = prepend + '/' + dirpart;
  }

  // Loop over the contents of this directory in the file.
  // Post-pone string object handling to happen after other
  // objects have been read in so we are guaranteed to have
  // histograms by the time we read in quality tests and tags.
  TKey* key{nullptr};
  TIter next{gDirectory->GetListOfKeys()};
  std::list<TObject*> delayed;
  while ((key = (TKey*)next())) {
    std::unique_ptr<TObject> obj{key->ReadObj()};
    if (dynamic_cast<TDirectory*>(obj.get())) {
      std::string subdir;
      subdir.reserve(curdir.size() + strlen(obj->GetName()) + 2);
      subdir += curdir;
      if (!curdir.empty())
        subdir += '/';
      subdir += obj->GetName();

      ntot +=
          readDirectory(file, overwrite, onlypath, prepend, subdir, stripdirs);
    }
    else if (skip)
      ;
    else if (dynamic_cast<TObjString*>(obj.get())) {
      delayed.push_back(obj.release());
    }
    else {
      if (verbose_ > 2)
        std::cout << "DQMStore: reading object '" << obj->GetName()
                  << "' of type '" << obj->IsA()->GetName() << "' from '"
                  << file->GetName() << "' into '" << dirpart << "'\n";

      makeDirectory(dirpart);
      if (extract(obj.get(), dirpart, overwrite, collateHistograms_))
        ++count;
    }
  }

  while (!delayed.empty()) {
    if (verbose_ > 2)
      std::cout << "DQMStore: reading object '" << delayed.front()->GetName()
                << "' of type '" << delayed.front()->IsA()->GetName()
                << "' from '" << file->GetName() << "' into '" << dirpart
                << "'\n";

    makeDirectory(dirpart);
    if (extract(delayed.front(), dirpart, overwrite, collateHistograms_))
      ++count;

    delete delayed.front();
    delayed.pop_front();
  }

  if (verbose_ > 1)
    std::cout << "DQMStore: read " << count << '/' << ntot
              << " objects from directory '" << dirpart << "'\n";

  return ntot + count;
}

/// public open/read root file <filename>, and copy MonitorElements;
/// if flag=true, overwrite identical MonitorElements (default: false);
/// if onlypath != "", read only selected directory
/// if prepend !="", prepend string to path
/// note: by default this method keeps the dir structure as in file
/// and does not update monitor element references!
bool DQMStore::open(std::string const& filename,
                    bool const overwrite /* = false */,
                    std::string const& onlypath /* ="" */,
                    std::string const& prepend /* ="" */,
                    OpenRunDirs const stripdirs /* =KeepRunDirs */,
                    bool const fileMustExist /* =true */)
{
  return readFile(filename, overwrite, onlypath, prepend, stripdirs,
                  fileMustExist);
}

/// public load root file <filename>, and copy MonitorElements;
/// overwrite identical MonitorElements (default: true);
/// set DQMStore.collateHistograms to true to sum several files
/// note: by default this method strips off run dir structure
bool DQMStore::load(std::string const& filename,
                    OpenRunDirs const stripdirs /* =StripRunDirs */,
                    bool const fileMustExist /* =true */)
{
  bool const overwrite{!collateHistograms_};
  if (verbose_) {
    std::cout << "DQMStore::load: reading from file '" << filename << "'\n";
    if (collateHistograms_)
      std::cout << "DQMStore::load: in collate mode   "
                << "\n";
    else
      std::cout << "DQMStore::load: in overwrite mode   "
                << "\n";
  }

  if (!s_rxpbfile.match(filename, 0, 0))
    return readFile(filename, overwrite, "", "", stripdirs, fileMustExist);
  else
    return readFilePB(filename, overwrite, "", "", stripdirs, fileMustExist);
}

/// private readFile <filename>, and copy MonitorElements;
/// if flag=true, overwrite identical MonitorElements (default: false);
/// if onlypath != "", read only selected directory
/// if prepend !="", prepend string to path
/// if StripRunDirs is set the run and run summary folders are erased.
bool DQMStore::readFile(std::string const& filename,
                        bool const overwrite /* = false */,
                        std::string const& onlypath /* ="" */,
                        std::string const& prepend /* ="" */,
                        OpenRunDirs const stripdirs /* =StripRunDirs */,
                        bool const fileMustExist /* =true */) // PRIVATE
{

  if (verbose_)
    std::cout << "DQMStore::readFile: reading from file '" << filename << "'\n";

  std::unique_ptr<TFile> f;

  try {
    f.reset(TFile::Open(filename.c_str()));
    if (!f.get() || f->IsZombie())
      raiseDQMError("DQMStore", "Failed to open file '%s'", filename.c_str());
  }
  catch (std::exception&) {
    if (fileMustExist)
      throw;
    else {
      if (verbose_)
        std::cout << "DQMStore::readFile: file '" << filename
                  << "' does not exist, continuing\n";
      return false;
    }
  }

  unsigned const n{
      readDirectory(f.get(), overwrite, onlypath, prepend, "", stripdirs)};
  f->Close();

  for (auto const& me : data_)
    const_cast<MonitorElement&>(me).updateQReportStats();

  if (verbose_) {
    std::cout << "DQMStore::open: successfully read " << n
              << " objects from file '" << filename << "'";
    if (!onlypath.empty())
      std::cout << " from directory '" << onlypath << "'";
    if (!prepend.empty())
      std::cout << " into directory '" << prepend << "'";
    std::cout << std::endl;
  }
  return true;
}

/** Extract the next serialised ROOT object from @a buf. Returns null
    if there are no more objects in the buffer, or a null pointer was
    serialised at this location. */
inline TObject* DQMStore::extractNextObject(TBufferFile& buf) const
{
  if (buf.Length() == buf.BufferSize())
    return nullptr;
  buf.InitMap();
  void* ptr = buf.ReadObjectAny(nullptr);
  return reinterpret_cast<TObject*>(ptr);
}

void DQMStore::get_info(dqmstorepb::ROOTFilePB::Histo const& h,
                        std::string& dirname, std::string& objname,
                        TObject** obj)
{
  size_t slash = h.full_pathname().rfind('/');
  size_t dirpos = (slash == std::string::npos ? 0 : slash);
  size_t namepos = (slash == std::string::npos ? 0 : slash + 1);
  dirname.assign(h.full_pathname(), 0, dirpos);
  objname.assign(h.full_pathname(), namepos, std::string::npos);
  TBufferFile buf(TBufferFile::kRead, h.size(),
                  (void*)h.streamed_histo().data(), kFALSE);
  buf.Reset();
  *obj = extractNextObject(buf);
  if (!*obj) {
    raiseDQMError("DQMStore", "Error reading element:'%s'",
                  h.full_pathname().c_str());
  }
}

bool DQMStore::readFilePB(std::string const& filename,
                          bool const overwrite /* = false */,
                          std::string const& onlypath /* ="" */,
                          std::string const& prepend /* ="" */,
                          OpenRunDirs const stripdirs /* =StripRunDirs */,
                          bool const fileMustExist /* =true */)
{
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::io::GzipInputStream;
  using google::protobuf::io::GzipOutputStream;
  using google::protobuf::io::CodedInputStream;
  using google::protobuf::io::ArrayInputStream;

  if (verbose_)
    std::cout << "DQMStore::readFile: reading from file '" << filename << "'\n";

  int filedescriptor;
  if ((filedescriptor = ::open(filename.c_str(), O_RDONLY)) == -1) {
    if (fileMustExist)
      raiseDQMError("DQMStore", "Failed to open file '%s'", filename.c_str());
    else if (verbose_)
      std::cout << "DQMStore::readFile: file '" << filename
                << "' does not exist, continuing\n";
    return false;
  }

  dqmstorepb::ROOTFilePB dqmstore_message;
  FileInputStream fin(filedescriptor);
  GzipInputStream input(&fin);
  CodedInputStream input_coded(&input);
  input_coded.SetTotalBytesLimit(1024 * 1024 * 1024, -1);
  if (!dqmstore_message.ParseFromCodedStream(&input_coded)) {
    raiseDQMError("DQMStore", "Fatal parsing file '%s'", filename.c_str());
    return false;
  }
  ::close(filedescriptor);

  for (int i = 0; i < dqmstore_message.histo_size(); i++) {
    std::string path;
    std::string objname;

    TObject* obj{nullptr};
    dqmstorepb::ROOTFilePB::Histo const& h = dqmstore_message.histo(i);
    get_info(h, path, objname, &obj);

    setCurrentFolder(path);
    if (obj) {
      /* Before calling the extract() check if histogram exists:
       * if it does - flags for the given monitor are already set (and merged)
       * else - set the flags after the histogram is created.
       */
      auto me = findObject(path, objname);

      /* Run histograms should be collated and not overwritten,
       * Lumi histograms should be overwritten (and collate flag is not checked)
       */
      bool const overwrite{h.flags() & DQMNet::DQM_PROP_LUMI};
      bool const collate{!(h.flags() & DQMNet::DQM_PROP_LUMI)};
      extract(static_cast<TObject*>(obj), path, overwrite, collate);

      if (me == nullptr) {
        me = findObject(path, objname);
        me->data_.flags = h.flags();
      }

      delete obj;
    }
  }

  cd();
  return true;
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/// delete directory and all contents;
/// delete directory (all contents + subfolders);
void DQMStore::rmdir(std::string const& path)
{
  std::string clean;
  std::string const* cleaned{nullptr};
  cleanTrailingSlashes(path, clean, cleaned);
  MonitorElement const proto{cleaned, std::string()};

  auto e = data_.end();
  auto i = data_.lower_bound(proto);
  while (i != e && isSubdirectory(*cleaned, *i->data_.dirname))
    data_.erase(i++);

  auto de = dirs_.end();
  auto di = dirs_.lower_bound(*cleaned);
  while (di != de && isSubdirectory(*cleaned, *di))
    dirs_.erase(di++);
}

/// remove all monitoring elements from directory;
void DQMStore::removeContents(std::string const& dir)
{
  MonitorElement const proto{&dir, std::string()};
  auto e = data_.end();
  auto i = data_.lower_bound(proto);
  while (i != e && isSubdirectory(dir, *i->data_.dirname))
    if (dir == *i->data_.dirname)
      data_.erase(i++);
    else
      ++i;
}

/// erase all monitoring elements in current directory (not including
/// subfolders);
void DQMStore::removeContents() { removeContents(pwd_); }

/// erase monitoring element in current directory
/// (opposite of book1D,2D,etc. action);
void DQMStore::removeElement(std::string const& name)
{
  removeElement(pwd_, name);
}

/// remove monitoring element from directory;
/// if warning = true, print message if element does not exist
void DQMStore::removeElement(std::string const& dir, std::string const& name,
                             bool warning /* = true */)
{
  MonitorElement const proto{&dir, name};
  auto pos = data_.find(proto);
  if (pos != data_.end())
    data_.erase(pos);
  else if (warning)
    std::cout << "DQMStore: WARNING: attempt to remove non-existent"
              << " monitor element '" << name << "' in '" << dir << "'\n";
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/// get QCriterion corresponding to <qtname>
/// (null pointer if QCriterion does not exist)
QCriterion* DQMStore::getQCriterion(std::string const& qtname) const
{
  auto i = qtests_.find(qtname);
  auto e = qtests_.cend();
  return (i == e ? nullptr : i->second);
}

/// create quality test with unique name <qtname> (analogous to ME name);
/// quality test can then be attached to ME with useQTest method
/// (<algo_name> must match one of known algorithms)
QCriterion* DQMStore::createQTest(std::string const& algoname,
                                  std::string const& qtname)
{
  if (qtests_.count(qtname))
    raiseDQMError("DQMStore", "Attempt to create duplicate quality test '%s'",
                  qtname.c_str());

  auto i = qalgos_.find(algoname);
  if (i == qalgos_.end())
    raiseDQMError("DQMStore", "Cannot create a quality test using unknown"
                              " algorithm '%s'",
                  algoname.c_str());

  QCriterion* qc = i->second(qtname);
  qc->setVerbose(verboseQT_);

  qtests_[qtname] = qc;
  return qc;
}

/// attach quality test <qtname> to directory contents
/// (need exact pathname without wildcards, e.g. A/B/C);
void DQMStore::useQTest(std::string const& dir, std::string const& qtname)
{
  // Clean the path
  std::string clean;
  std::string const* cleaned{nullptr};
  cleanTrailingSlashes(dir, clean, cleaned);

  // Validate the path.
  if (cleaned->find_first_not_of(s_safe) != std::string::npos)
    raiseDQMError("DQMStore", "Monitor element path name '%s'"
                              " uses unacceptable characters",
                  cleaned->c_str());

  // Redirect to the pattern match version.
  useQTestByMatch(*cleaned + "/*", qtname);
}

/// attach quality test <qc> to monitor elements matching <pattern>.
int DQMStore::useQTestByMatch(std::string const& pattern,
                              std::string const& qtname)
{
  QCriterion* qc = getQCriterion(qtname);
  if (!qc)
    raiseDQMError("DQMStore", "Cannot apply non-existent quality test '%s'",
                  qtname.c_str());

  auto* fm = new fastmatch{pattern};

  // Record the test for future reference.
  QTestSpec const qts{fm, qc};
  qtestspecs_.push_back(qts);

  // Apply the quality test.
  std::string path;
  int cases{0};
  for (auto const& me : data_) {
    path.clear();
    mergePath(path, *me.data_.dirname, me.data_.objname);
    if (fm->match(path)) {
      ++cases;
      const_cast<MonitorElement&>(me).addQReport(qts.second);
    }
  }

  // return the number of matched cases
  return cases;
}
/// run quality tests (also finds updated contents in last monitoring cycle,
/// including newly added content)
void DQMStore::runQTests()
{
  if (verbose_ > 0)
    std::cout << "DQMStore: running runQTests() with reset = "
              << (reset_ ? "true" : "false") << std::endl;

  // Apply quality tests to each monitor element, skipping references.
  auto mi = data_.begin();
  auto me = data_.end();
  for (; mi != me; ++mi)
    if (!isSubdirectory(s_referenceDirName, *mi->data_.dirname))
      const_cast<MonitorElement&>(*mi).runQTests();

  reset_ = false;
}

/// get "global" folder <path> status (one of:STATUS_OK, WARNING, ERROR, OTHER);
/// returns most sever error, where ERROR > WARNING > OTHER > STATUS_OK;
/// see Core/interface/QTestStatus.h for details on "OTHER"
int DQMStore::getStatus(std::string const& path /* = "" */) const
{
  std::string clean;
  std::string const* cleaned{nullptr};
  cleanTrailingSlashes(path, clean, cleaned);

  int status{dqm::qstatus::STATUS_OK};
  for (auto const& me : data_) {
    if (!cleaned->empty() && !isSubdirectory(*cleaned, *me.data_.dirname))
      continue;

    if (me.hasError())
      return dqm::qstatus::ERROR;
    else if (me.hasWarning())
      status = dqm::qstatus::WARNING;
    else if (status < dqm::qstatus::WARNING && me.hasOtherReport())
      status = dqm::qstatus::OTHER;
  }
  return status;
}

//////////////////////////////////////////////////////////////////////
/// reset contents (does not erase contents permanently)
/// (makes copy of current contents; will be subtracted from future contents)
void DQMStore::softReset(MonitorElement* me)
{
  if (me)
    me->softReset();
}

// reverts action of softReset
void DQMStore::disableSoftReset(MonitorElement* me)
{
  if (me)
    me->disableSoftReset();
}

/// if true, will accumulate ME contents (over many periods)
/// until method is called with flag = false again
void DQMStore::setAccumulate(MonitorElement* me, bool flag)
{
  if (me)
    me->setAccumulate(flag);
}

//////////////////////////////////////////////////////////////////////
void DQMStore::showDirStructure() const
{
  std::vector<std::string> contents;
  getContents(contents);

  std::cout
      << " ------------------------------------------------------------\n"
      << "                    Directory structure:                     \n"
      << " ------------------------------------------------------------\n";

  std::copy(contents.begin(), contents.end(),
            std::ostream_iterator<std::string>(std::cout, "\n"));

  std::cout
      << " ------------------------------------------------------------\n";
}

//////////////////////////////////////////////////////////////////////
// check if the collate option is active on the DQMStore
bool DQMStore::isCollate() const { return collateHistograms_; }

//////////////////////////////////////////////////////////////////////
// check if the monitor element is in auto-collation folder
bool DQMStore::isCollateME(MonitorElement* me) const
{
  return me && isSubdirectory(s_collateDirName, *me->data_.dirname);
}

//////////////////////////////////////////////////////////////////////
/* Invoke this method after flushing all recently changed monitoring.
   Clears updated flag on all MEs and calls their Reset() method. */
void DQMStore::scaleElements()
{
  if (scaleFlag_ == 0.0)
    return;
  if (verbose_ > 0)
    std::cout << " =========== "
              << " ScaleFlag " << scaleFlag_ << std::endl;
  double factor = scaleFlag_;
  int events{1};
  if (dirExists("Info/EventInfo")) {
    if (scaleFlag_ == -1.0) {
      MonitorElement* scale_me = get("Info/EventInfo/ScaleFactor");
      if (scale_me && scale_me->kind() == MonitorElement::DQM_KIND_REAL)
        factor = scale_me->getFloatValue();
    }
    MonitorElement* event_me = get("Info/EventInfo/processedEvents");
    if (event_me && event_me->kind() == MonitorElement::DQM_KIND_INT)
      events = event_me->getIntValue();
  }
  factor = factor / (events * 1.0);

  for (auto const& me : data_) {
    switch (me.kind()) {
    case MonitorElement::DQM_KIND_TH1F: {
      me.getTH1F()->Scale(factor);
      break;
    }
    case MonitorElement::DQM_KIND_TH1S: {
      me.getTH1S()->Scale(factor);
      break;
    }
    case MonitorElement::DQM_KIND_TH1D: {
      me.getTH1D()->Scale(factor);
      break;
    }
    case MonitorElement::DQM_KIND_TH2F: {
      me.getTH2F()->Scale(factor);
      break;
    }
    case MonitorElement::DQM_KIND_TH2S: {
      me.getTH2S()->Scale(factor);
      break;
    }
    case MonitorElement::DQM_KIND_TH2D: {
      me.getTH2D()->Scale(factor);
      break;
    }
    case MonitorElement::DQM_KIND_TH3F: {
      me.getTH3F()->Scale(factor);
      break;
    }
    case MonitorElement::DQM_KIND_TPROFILE: {
      me.getTProfile()->Scale(factor);
      break;
    }
    case MonitorElement::DQM_KIND_TPROFILE2D: {
      me.getTProfile2D()->Scale(factor);
      break;
    }
    default:
      if (verbose_ > 0)
        std::cout << " The DQM object '" << me.getFullname()
                  << "' is not scalable object " << std::endl;
      continue;
    }
  }
}
