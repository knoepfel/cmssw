#ifndef DQMServices_Core_DQMStore_h
#define DQMServices_Core_DQMStore_h

#if __GNUC__ && !defined DQM_DEPRECATED
#define DQM_DEPRECATED __attribute__((deprecated))
#endif

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iosfwd>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <cxxabi.h>
#include <execinfo.h>

#include <classlib/utils/Regexp.h>

#include "DQMServices/Core/interface/DQMDefinitions.h"
#include "DQMServices/Core/interface/ConcurrentMonitorElement.h"
#include "DQMServices/Core/interface/MonitorElement.h" // Rethink this?
#include "tbb/concurrent_unordered_map.h"

namespace edm {
  class DQMHttpSource;
  class ParameterSet;
  class ActivityRegistry;
  class GlobalContext;
}
namespace dqmstorepb {
  class ROOTFilePB;
  class ROOTFilePB_Histo;
}

class MonitorElement;
class QCriterion;
class TFile;
class TBufferFile;
class TObject;
class TH1;
class TObjString;
class TH1F;
class TH1S;
class TH1D;
class TH2F;
class TH2S;
class TH2D;
class TH3F;
class TProfile;
class TProfile2D;
class TNamed;

/** Implements RegEx patterns which occur often in a high-performant
    mattern. For all other expressions, the full RegEx engine is used.
    Note: this class can only be used for lat::Regexp::Wildcard-like
    patterns.  */
class fastmatch {
private:
  enum MatchingHeuristicEnum { UseFull, OneStarStart, OneStarEnd, TwoStar };

public:
  fastmatch(std::string const& fastString);

  bool match(std::string const& s) const;

private:
  // checks if two strings are equal, starting at the back of the strings
  bool compare_strings_reverse(std::string const& pattern,
                               std::string const& input) const;
  // checks if two strings are equal, starting at the front of the strings
  bool compare_strings(std::string const& pattern,
                       std::string const& input) const;

  std::unique_ptr<lat::Regexp> regexp_;
  std::string fastString_;
  MatchingHeuristicEnum matching_;
};

class DQMStore {
public:
  enum SaveReferenceTag {
    SaveWithoutReference,
    SaveWithReference,
    SaveWithReferenceForQTest
  };
  enum OpenRunDirs { KeepRunDirs, StripRunDirs };

  using module_id_t = uint32_t;//decltype(std::declval<ModuleDescription>().id());

  class IBooker {
  public:
    friend class DQMStore;

    // for the supported syntaxes, see the declarations of DQMStore::bookString
    template <typename... Args> MonitorElement* bookString(Args&&... args)
    {
      return owner_->bookString(std::forward<Args>(args)..., id_);
    }

    // for the supported syntaxes, see the declarations of DQMStore::bookInt
    template <typename... Args> MonitorElement* bookInt(Args&&... args)
    {
      return owner_->bookInt(std::forward<Args>(args)...);
    }

    // for the supported syntaxes, see the declarations of DQMStore::bookFloat
    template <typename... Args> MonitorElement* bookFloat(Args&&... args)
    {
      return owner_->bookFloat(std::forward<Args>(args)...);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book1D
    template <typename... Args> MonitorElement* book1D(Args&&... args)
    {
      return owner_->book1D(std::forward<Args>(args)...);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book1S
    template <typename... Args> MonitorElement* book1S(Args&&... args)
    {
      return owner_->book1S(std::forward<Args>(args)...);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book1DD
    template <typename... Args> MonitorElement* book1DD(Args&&... args)
    {
      return owner_->book1DD(std::forward<Args>(args)...);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book2D
    template <typename... Args> MonitorElement* book2D(Args&&... args)
    {
      return owner_->book2D(std::forward<Args>(args)...);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book2S
    template <typename... Args> MonitorElement* book2S(Args&&... args)
    {
      return owner_->book2S(std::forward<Args>(args)...);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book2DD
    template <typename... Args> MonitorElement* book2DD(Args&&... args)
    {
      return owner_->book2DD(std::forward<Args>(args)...);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book3D
    template <typename... Args> MonitorElement* book3D(Args&&... args)
    {
      return owner_->book3D(std::forward<Args>(args)...);
    }

    // for the supported syntaxes, see the declarations of DQMStore::bookProfile
    template <typename... Args> MonitorElement* bookProfile(Args&&... args)
    {
      return owner_->bookProfile(std::forward<Args>(args)...);
    }

    // for the supported syntaxes, see the declarations of
    // DQMStore::bookProfile2D
    template <typename... Args> MonitorElement* bookProfile2D(Args&&... args)
    {
      return owner_->bookProfile2D(std::forward<Args>(args)...);
    }

    void cd();
    void cd(const std::string& dir);
    void setCurrentFolder(const std::string& fullpath);
    void goUp();
    const std::string& pwd();
    void tag(MonitorElement*, unsigned int);
    void tagContents(const std::string&, unsigned int);

  private:
    explicit IBooker(DQMStore* store,
                     uint32_t const run = {},
                     uint32_t const lumi = {},
                     uint32_t const streamId = {},
                     module_id_t const moduleId = {}) :
      owner_{store}, id_{run, lumi, streamId, moduleId}
    {
      assert(store);
    }

    IBooker(IBooker const&) = delete;
    IBooker& operator=(IBooker const&) = delete;

   private:
    // Embedded classes do not natively own a pointer to the embedding
    // class. We therefore need to store a pointer to the main
    // DQMStore instance (owner_).
    DQMStore* owner_;
    MonitorElement::Identifier id_;
  }; // IBooker

  class ConcurrentBooker : public IBooker
  {
  public:
    friend class DQMStore;

    // for the supported syntaxes, see the declarations of DQMStore::bookString
    template <typename... Args>
    ConcurrentMonitorElement bookString(Args && ... args) {
      MonitorElement* me = IBooker::bookString(std::forward<Args>(args)...);
      return ConcurrentMonitorElement(me);
    }

    // for the supported syntaxes, see the declarations of DQMStore::bookInt
    template <typename... Args>
    ConcurrentMonitorElement bookInt(Args && ... args) {
      MonitorElement* me = IBooker::bookInt(std::forward<Args>(args)...);
      return ConcurrentMonitorElement(me);
    }

    // for the supported syntaxes, see the declarations of DQMStore::bookFloat
    template <typename... Args>
    ConcurrentMonitorElement bookFloat(Args && ... args) {
      MonitorElement* me = IBooker::bookFloat(std::forward<Args>(args)...);
      return ConcurrentMonitorElement(me);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book1D
    template <typename... Args>
    ConcurrentMonitorElement book1D(Args && ... args) {
      MonitorElement* me = IBooker::book1D(std::forward<Args>(args)...);
      return ConcurrentMonitorElement(me);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book1S
    template <typename... Args>
    ConcurrentMonitorElement book1S(Args && ... args) {
      MonitorElement* me = IBooker::book1S(std::forward<Args>(args)...);
      return ConcurrentMonitorElement(me);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book1DD
    template <typename... Args>
    ConcurrentMonitorElement book1DD(Args && ... args) {
      MonitorElement* me = IBooker::book1DD(std::forward<Args>(args)...);
      return ConcurrentMonitorElement(me);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book2D
    template <typename... Args>
    ConcurrentMonitorElement book2D(Args && ... args) {
      MonitorElement* me = IBooker::book2D(std::forward<Args>(args)...);
      return ConcurrentMonitorElement(me);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book2S
    template <typename... Args>
    ConcurrentMonitorElement book2S(Args && ... args) {
      MonitorElement* me = IBooker::book2S(std::forward<Args>(args)...);
      return ConcurrentMonitorElement(me);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book2DD
    template <typename... Args>
    ConcurrentMonitorElement book2DD(Args && ... args) {
      MonitorElement* me = IBooker::book2DD(std::forward<Args>(args)...);
      return ConcurrentMonitorElement(me);
    }

    // for the supported syntaxes, see the declarations of DQMStore::book3D
    template <typename... Args>
    ConcurrentMonitorElement book3D(Args && ... args) {
      MonitorElement* me = IBooker::book3D(std::forward<Args>(args)...);
      return ConcurrentMonitorElement(me);
    }

    // for the supported syntaxes, see the declarations of DQMStore::bookProfile
    template <typename... Args>
    ConcurrentMonitorElement bookProfile(Args && ... args) {
      MonitorElement* me = IBooker::bookProfile(std::forward<Args>(args)...);
      return ConcurrentMonitorElement(me);
    }

    // for the supported syntaxes, see the declarations of DQMStore::bookProfile2D
    template <typename... Args>
    ConcurrentMonitorElement bookProfile2D(Args && ... args) {
      MonitorElement* me = IBooker::bookProfile2D(std::forward<Args>(args)...);
      return ConcurrentMonitorElement(me);
    }

  private:
    explicit ConcurrentBooker(DQMStore * store) :
      IBooker(store)
    { }

    ConcurrentBooker() = delete;
    ConcurrentBooker(ConcurrentBooker const&) = delete;
    ConcurrentBooker(ConcurrentBooker &&) = delete;
    ConcurrentBooker& operator= (ConcurrentBooker const&) = delete;
    ConcurrentBooker& operator= (ConcurrentBooker &&) = delete;

    ~ConcurrentBooker() = default;
  };

  class IGetter
  {
  public:
    friend class DQMStore;

    // for the supported syntaxes, see the declarations of DQMStore::getContents
    template <typename... Args>
    std::vector<MonitorElement*> getContents(Args&&... args)
    {
      return owner_->getContents(std::forward<Args>(args)...);
    }

    // for the supported syntaxes, see the declarations of
    // DQMStore::removeElements
    template <typename... Args> void removeElement(Args&&... args)
    {
      return owner_->removeElement(std::forward<Args>(args)...);
    }

    std::vector<MonitorElement*> getAllContents(std::string const& path,
                                                uint32_t runNumber = 0,
                                                uint32_t lumi = 0);
    MonitorElement* get(std::string const& path);

    // same as get, throws an exception if histogram not found
    MonitorElement* getElement(std::string const& path);

    std::vector<std::string> getSubdirs();
    std::vector<std::string> getMEs();
    bool containsAnyMonitorable(std::string const& path);
    bool dirExists(std::string const& path);
    void cd();
    void cd(std::string const& dir);
    void setCurrentFolder(std::string const& fullpath);

  private:
    explicit IGetter(DQMStore* store) : owner_{store} { assert(store); }

    IGetter(IGetter const&) = delete;
    IGetter& operator=(IGetter const&) = delete;

   private:
    // Embedded classes do not natively own a pointer to the embedding
    // class. We therefore need to store a pointer to the main
    // DQMStore instance (owner_).
    DQMStore* owner_;
  }; // IGetter

  // Template function to be used inside each DQM Modules' lambda
  // functions to book MonitorElements into the DQMStore. The function
  // calls whatever user-supplied code via the function f. The latter
  // is passed the instance of the IBooker class (owned by the *only*
  // DQMStore instance), that is capable of booking MonitorElements
  // into the DQMStore via a public API. The central mutex is acquired
  // *before* invoking and automatically released upon returns.
  template <typename iFunc>
  void bookTransaction(iFunc f, uint32_t run, uint32_t streamId,
                       module_id_t moduleId)
  {
    std::lock_guard<std::mutex> guard(book_mutex_);
    IBooker booker{this, run, streamId, moduleId};
    f(booker);
  }

  // Similar function used to book "global" histograms via the
  // ConcurrentMonitorElement interface.
  template <typename iFunc>
  void bookConcurrentTransaction(iFunc f, uint32_t run) {
    std::lock_guard<std::mutex> guard(book_mutex_);
    /* Even if enableMultiThread_ is enabled, keep the streamId_
       and moduleId_ to 0, since we want to book global histograms. */
    if (enableMultiThread_) {
      run_ = run;
    }
    ConcurrentBooker booker(this);
    f(booker);

    /* Set back to 0 the run_ in case we run in mixed conditions
       with DQMEDAnalyzers and legacy modules */
    if (enableMultiThread_) {
      run_ = 0;
    }
  }

  // Signature needed in the harvesting where the booking is done
  // in the endJob. No handles to the run there. Two arguments ensure
  // the capability of booking and getting. The method relies on the
  // initialization of run, stream and module ID to 0. The mutex
  // is not needed.
  template <typename iFunc> void meBookerGetter(iFunc f)
  {
    IBooker booker {this};
    IGetter getter {this};
    f(booker, getter);
  }

  // Signature needed in the harvesting where it might be needed to get
  // the LS based histograms. Handle to the Lumi and to the iSetup are
  // available.
  // No need to book anything there. The method relies on the
  // initialization of run, stream and module ID to 0. The mutex
  // is not needed.
  template <typename iFunc> void meGetter(iFunc f)
  {
    IGetter getter {this};
    f(getter);
  }

  //-------------------------------------------------------------------------
  // ---------------------- Constructors ------------------------------------
  DQMStore(edm::ParameterSet const& pset, edm::ActivityRegistry&);
  DQMStore(edm::ParameterSet const& pset);
  ~DQMStore();

  //-------------------------------------------------------------------------
  void setVerbose(unsigned level);

  // ---------------------- public navigation -------------------------------
  std::string const& pwd() const;
  void cd();
  void cd(std::string const& subdir);
  void setCurrentFolder(std::string const& fullpath);
  void goUp();

  bool dirExists(std::string const& path) const;

  //-------------------------------------------------------------------------
  // ---------------------- public ME booking -------------------------------

  MonitorElement* bookInt(std::string const& name);
  MonitorElement* bookFloat(std::string const& name);
  MonitorElement* bookString(std::string const& name, std::string const& value, MonitorElement::Identifier const& identifier = {});

  // char_string is a type that enables implicit conversion from an
  // 'std::string' to a 'char const*' object.  It is a utility to
  // enable a simpler interface with ROOT.
  class char_string {
  public:
    char_string(char const* str) : str_{str} {}
    char_string(std::string const& str) : str_{str.c_str()} {}
    operator char const*() const { return str_; }
    operator std::string() const { return std::string{str_}; }
  private:
    char const* str_;
  };

  MonitorElement* book1D(char_string const& name, char_string const& title,
                         int nchX, double lowX, double highX);
  MonitorElement* book1D(char_string const& name, char_string const& title,
                         int nchX, float const* xbinsize);
  MonitorElement* book1D(char_string const& name, TH1F* h);

  MonitorElement* book1S(char_string const& name, char_string const& title,
                         int nchX, double lowX, double highX);
  MonitorElement* book1S(char_string const& name, char_string const& title,
                         int nchX, float const* xbinsize);
  MonitorElement* book1S(char_string const& name, TH1S* h);

  MonitorElement* book1DD(char_string const& name, char_string const& title,
                          int nchX, double lowX, double highX);
  MonitorElement* book1DD(char_string const& name, char_string const& title,
                          int nchX, float const* xbinsize);
  MonitorElement* book1DD(char_string const& name, TH1D* h);

  MonitorElement* book2D(char_string const& name, char_string const& title,
                         int nchX, double lowX, double highX, int nchY,
                         double lowY, double highY);
  MonitorElement* book2D(char_string const& name, char_string const& title,
                         int nchX, float const* xbinsize, int nchY,
                         float const* ybinsize);
  MonitorElement* book2D(char_string const& name, TH2F* h);

  MonitorElement* book2S(char_string const& name, char_string const& title,
                         int nchX, double lowX, double highX, int nchY,
                         double lowY, double highY);
  MonitorElement* book2S(char_string const& name, char_string const& title,
                         int nchX, float const* xbinsize, int nchY,
                         float const* ybinsize);
  MonitorElement* book2S(char_string const& name, TH2S* h);

  MonitorElement* book2DD(char_string const& name, char_string const& title,
                          int nchX, double lowX, double highX, int nchY,
                          double lowY, double highY);
  MonitorElement* book2DD(char_string const& name, char_string const& title,
                          int nchX, float const* xbinsize, int nchY,
                          float const* ybinsize);
  MonitorElement* book2DD(char_string const& name, TH2D* h);

  MonitorElement* book3D(char_string const& name, char_string const& title,
                         int nchX, double lowX, double highX, int nchY,
                         double lowY, double highY, int nchZ, double lowZ,
                         double highZ);
  MonitorElement* book3D(char_string const& name, TH3F* h);

  MonitorElement* bookProfile(char_string const& name, char_string const& title,
                              int nchX, double lowX, double highX, int nchY,
                              double lowY, double highY,
                              char const* option = "s");
  MonitorElement* bookProfile(char_string const& name, char_string const& title,
                              int nchX, double lowX, double highX, double lowY,
                              double highY, char const* option = "s");
  MonitorElement* bookProfile(char_string const& name, char_string const& title,
                              int nchX, const double* xbinsize, int nchY,
                              double lowY, double highY,
                              char const* option = "s");
  MonitorElement* bookProfile(char_string const& name, char_string const& title,
                              int nchX, const double* xbinsize, double lowY,
                              double highY, char const* option = "s");
  MonitorElement* bookProfile(char_string const& name, TProfile* h);

  MonitorElement* bookProfile2D(char_string const& name,
                                char_string const& title, int nchX, double lowX,
                                double highX, int nchY, double lowY,
                                double highY, int nchZ, double lowZ,
                                double highZ, char const* option = "s");
  MonitorElement* bookProfile2D(char_string const& name,
                                char_string const& title, int nchX, double lowX,
                                double highX, int nchY, double lowY,
                                double highY, double lowZ, double highZ,
                                char const* option = "s");
  MonitorElement* bookProfile2D(char_string const& name, TProfile2D* h);

  //-------------------------------------------------------------------------
  // ---------------------- public tagging ----------------------------------
  void tag(MonitorElement* me, unsigned int myTag);
  void tag(std::string const& path, unsigned int myTag);
  void tagContents(std::string const& path, unsigned int myTag);
  void tagAllContents(std::string const& path, unsigned int myTag);

  //-------------------------------------------------------------------------
  // ---------------------- public ME getters -------------------------------
  std::vector<std::string> getSubdirs() const;
  std::vector<std::string> getMEs() const;
  bool containsAnyMonitorable(std::string const& path) const;

  MonitorElement* get(std::string const& path) const;
  std::vector<MonitorElement*> get(unsigned int tag) const;
  std::vector<MonitorElement*> getContents(std::string const& path) const;
  std::vector<MonitorElement*> getContents(std::string const& path,
                                           unsigned int tag) const;
  void getContents(std::vector<std::string>& into,
                   bool showContents = true) const;

  // ---------------------- softReset methods -------------------------------
  void softReset(MonitorElement* me);
  void disableSoftReset(MonitorElement* me);

  // ---------------------- Public deleting ---------------------------------
  void rmdir(std::string const& fullpath);
  void removeContents();
  void removeContents(std::string const& dir);
  void removeElement(std::string const& name);
  void removeElement(std::string const& dir, std::string const& name,
                     bool warning = true);

  // ------------------------------------------------------------------------
  // ---------------------- public I/O --------------------------------------
  void save(std::string const& filename,
            std::string const& path = "",
            std::string const& pattern = "",
            std::string const& rewrite = "",
            uint32_t run = 0,
            uint32_t lumi = 0,
            SaveReferenceTag ref = SaveWithReference,
            int minStatus = dqm::qstatus::STATUS_OK,
            std::string const& fileupdate = "RECREATE");
  void savePB(std::string const& filename,
              std::string const& path = "",
              uint32_t run = 0,
              uint32_t lumi = 0);
  bool open(std::string const& filename,
            bool overwrite = false,
            std::string const& path ="",
            std::string const& prepend = "",
            OpenRunDirs stripdirs = KeepRunDirs,
            bool fileMustExist = true);
  bool load(std::string const& filename,
            OpenRunDirs stripdirs = StripRunDirs,
            bool fileMustExist = true);
  bool mtEnabled() const { return enableMultiThread_; };


  // -------------------------------------------------------------------------
  // ---------------------- Public print methods -----------------------------
  void showDirStructure() const;

  // ---------------------- Public check options -----------------------------
  bool isCollate() const;

  // -------------------------------------------------------------------------
  // ---------------------- Quality Test methods -----------------------------
  QCriterion* getQCriterion(std::string const& qtname) const;
  QCriterion* createQTest(std::string const& algoname,
                          std::string const& qtname);
  void useQTest(std::string const& dir, std::string const& qtname);
  int useQTestByMatch(std::string const& pattern, std::string const& qtname);
  void runQTests();
  int getStatus(std::string const& path = "") const;
  void scaleElements();

private:
  // ---------------- Navigation -----------------------
  bool cdInto(std::string const& path) const;

  // ------------------- Reference ME -------------------------------
  bool isCollateME(MonitorElement* me) const;

  // ------------------- Private "getters" ------------------------------
  bool readFilePB(std::string const& filename, bool overwrite = false,
                  std::string const& path = "", std::string const& prepend = "",
                  OpenRunDirs stripdirs = StripRunDirs,
                  bool fileMustExist = true);
  bool readFile(std::string const& filename, bool overwrite = false,
                std::string const& path = "", std::string const& prepend = "",
                OpenRunDirs stripdirs = StripRunDirs,
                bool fileMustExist = true);

  void makeDirectory(std::string const& path);
  unsigned int readDirectory(TFile* file, bool overwrite,
                             std::string const& path,
                             std::string const& prepend,
                             std::string const& curdir, OpenRunDirs stripdirs);

  MonitorElement* findObject(std::string const& dir, std::string const& name,
                             uint32_t const run = 0,
                             uint32_t const lumi = 0,
                             uint32_t const streamId = 0,
                             module_id_t const moduleId = 0) const;

  void get_info(dqmstorepb::ROOTFilePB_Histo const&, std::string& dirname,
                std::string& objname, TObject** obj);

public:
  void getAllTags(std::vector<std::string>& into) const;
  std::vector<MonitorElement*> getAllContents(std::string const& path,
                                              uint32_t runNumber = 0,
                                              uint32_t lumi = 0) const;
  std::vector<MonitorElement*> getMatchingContents(
      std::string const& pattern,
      lat::Regexp::Syntax syntaxType = lat::Regexp::Wildcard) const;

  // Multithread SummaryCache manipulations
  void mergeAndResetMEsRunSummaryCache(uint32_t run, uint32_t streamId,
                                       module_id_t moduleId);
  void mergeAndResetMEsLuminositySummaryCache(uint32_t run, uint32_t lumi,
                                              uint32_t streamId,
                                              module_id_t moduleId);

  void deleteUnusedLumiHistograms(uint32_t run, uint32_t lumi);

private:
  // ---------------- Miscellaneous -----------------------------
  void        initializeFrom(const edm::ParameterSet&);
  void        reset();
  void        forceReset();
  void        postGlobalBeginLumi(const edm::GlobalContext&);

  bool        extract(TObject *obj, const std::string &dir, bool overwrite, bool collateHistograms);
  TObject *   extractNextObject(TBufferFile&) const;

  // ---------------------- Booking ------------------------------------
  MonitorElement* initialise(MonitorElement *me, const std::string &path);
  MonitorElement* book_(const std::string &dir,
                        const std::string &name,
                        const char *context);
  template <class HISTO, class COLLATE>
  MonitorElement* book_(const std::string &dir,
                        const std::string &name,
                        const char *context,
                        int kind, HISTO *h, COLLATE collate);

  MonitorElement* bookInt_(const std::string &dir, const std::string &name);
  MonitorElement* bookFloat_(const std::string &dir, const std::string &name);
  MonitorElement* bookString_(const std::string &dir, const std::string &name, const std::string &value, MonitorElement::Identifier const&);
  MonitorElement* book1D_(const std::string &dir, const std::string &name, TH1F *h);
  MonitorElement* book1S_(const std::string &dir, const std::string &name, TH1S *h);
  MonitorElement* book1DD_(const std::string &dir, const std::string &name, TH1D *h);
  MonitorElement* book2D_(const std::string &dir, const std::string &name, TH2F *h);
  MonitorElement* book2S_(const std::string &dir, const std::string &name, TH2S *h);
  MonitorElement* book2DD_(const std::string &dir, const std::string &name, TH2D *h);
  MonitorElement* book3D_(const std::string &dir, const std::string &name, TH3F *h);
  MonitorElement* bookProfile_(const std::string &dir, const std::string &name, TProfile *h);
  MonitorElement* bookProfile2D_(const std::string &dir, const std::string &name, TProfile2D *h);

  static bool checkBinningMatches(MonitorElement *me, TH1 *h, unsigned verbose);

  static void collate1D(MonitorElement *me, TH1F *h, unsigned verbose);
  static void collate1S(MonitorElement *me, TH1S *h, unsigned verbose);
  static void collate1DD(MonitorElement *me, TH1D *h, unsigned verbose);
  static void collate2D(MonitorElement *me, TH2F *h, unsigned verbose);
  static void collate2S(MonitorElement *me, TH2S *h, unsigned verbose);
  static void collate2DD(MonitorElement *me, TH2D *h, unsigned verbose);
  static void collate3D(MonitorElement *me, TH3F *h, unsigned verbose);
  static void collateProfile(MonitorElement *me, TProfile *h, unsigned verbose);
  static void collateProfile2D(MonitorElement *me, TProfile2D *h, unsigned verbose);

  // --- Operations on MEs that are normally reset at end of monitoring cycle
  // ---
  void setAccumulate(MonitorElement* me, bool flag);

  void print_trace(std::string const& dir, std::string const& name);

  // ----------------------- Unavailable ---------------------------------------
  DQMStore(DQMStore const&) = delete;
  DQMStore& operator=(DQMStore const&) = delete;

  //-------------------------------------------------------------------------------
  //-------------------------------------------------------------------------------
  using QTestSpec  = std::pair<fastmatch *, QCriterion *>;
  using QTestSpecs = std::list<QTestSpec>;
  using MEMap      = std::set<MonitorElement>;
  using SafeMEMap  = std::vector<tbb::concurrent_unordered_map<module_id_t, std::set<MonitorElement>>>;
  using QCMap      = std::map<std::string, QCriterion *>;
  using QAMap      = std::map<std::string, std::function<QCriterion*(std::string const&)>>;

  // ------------------------ private I/O helpers ------------------------------
  void saveMonitorElementToPB(MonitorElement const& me,
                              dqmstorepb::ROOTFilePB & file);
  void saveMonitorElementRangeToPB(std::string const& dir,
                                   unsigned int run,
                                   MEMap::const_iterator begin,
                                   MEMap::const_iterator end,
                                   dqmstorepb::ROOTFilePB & file,
                                   unsigned int & counter);
  void saveMonitorElementToROOT(MonitorElement const& me,
                                TFile & file);
  void saveMonitorElementRangeToROOT(std::string const& dir,
                                     std::string const& refpath,
                                     SaveReferenceTag ref,
                                     int minStatus,
                                     unsigned int run,
                                     MEMap::const_iterator begin,
                                     MEMap::const_iterator end,
                                     TFile & file,
                                     unsigned int & counter);

  unsigned verbose_{1};
  unsigned verboseQT_{1};
  bool reset_{false};
  double scaleFlag_;
  bool collateHistograms_{false};
  bool enableMultiThread_{false};
  bool LSbasedMode_;
  bool forceResetOnBeginLumi_{false};
  std::string readSelectedDirectory_{};
  uint32_t run_{};
  uint32_t streamId_{};
  module_id_t moduleId_{};
  std::ofstream* stream_{nullptr};

  std::string pwd_{};
  MEMap data_;
  SafeMEMap safe_data_;
  std::set<std::string> dirs_;

  QCMap qtests_;
  QAMap qalgos_;
  QTestSpecs qtestspecs_;

  std::mutex book_mutex_;

  friend class edm::DQMHttpSource;
  friend class DQMService;
  friend class DQMNet;
  friend class DQMArchiver;
  friend class DQMStoreExample; // for get{All,Matching}Contents -- sole user of
                                // this method!
  friend class DQMRootOutputModule;
  friend class DQMRootSource;
  friend class DQMFileSaver;
  friend class MEtoEDMConverter;
};

#endif // DQMServices_Core_DQMStore_h
