/* \class PFJetSelector
 *
 * Selects jets with a configurable string-based cut,
 * and also writes out the constituents of the jet
 * into a separate collection.
 *
 * \author: Sal Rappoccio
 *
 *
 * for more details about the cut syntax, see the documentation
 * page below:
 *
 *   https://twiki.cern.ch/twiki/bin/view/CMS/SWGuidePhysicsCutParser
 *
 *
 */


#include "FWCore/Framework/interface/EDFilter.h"

#include "DataFormats/JetReco/interface/Jet.h"
#include "DataFormats/JetReco/interface/PFJet.h"
#include "DataFormats/PatCandidates/interface/Jet.h"
#include "DataFormats/PatCandidates/interface/PackedCandidate.h"
#include "DataFormats/JetReco/interface/CaloJet.h"

#include "FWCore/Framework/interface/MakerMacros.h"
#include "CommonTools/UtilAlgos/interface/StringCutObjectSelector.h"
#include "FWCore/Framework/interface/Event.h"

template <class T, typename C = std::vector<typename T::ConstituentTypeFwdPtr>>
class JetConstituentSelector : public edm::EDFilter {
public:

  using JetsOutput = std::vector<T>;
  using ConstituentsOutput = C;

  JetConstituentSelector(edm::ParameterSet const& params) :
    srcToken_{consumes<typename edm::View<T>>(params.getParameter<edm::InputTag>("src"))},
    selector_{params.getParameter<std::string>("cut")}
  {
    produces<JetsOutput>();
    produces<ConstituentsOutput>("constituents");
  }

  bool filter(edm::Event& iEvent, const edm::EventSetup& iSetup) override
  {
    auto jets = std::make_unique<JetsOutput>();
    auto candsOut = std::make_unique<ConstituentsOutput>();

    edm::Handle<typename edm::View<T>> h_jets;
    iEvent.getByToken(srcToken_, h_jets);

    // Now set the Ptrs with the orphan handles.
    for (auto const& jet : *h_jets) {
      // Check the selection
      if (selector_(jet)) {
        // Add the jets that pass to the output collection
        jets->push_back(jet);
        for (unsigned int ida = 0; ida < jet.numberOfDaughters(); ++ida) {
          candsOut->emplace_back(jet.daughterPtr(ida), jet.daughterPtr(ida));
        }
      }
    }

    // put  in Event
    iEvent.put(std::move(jets));
    iEvent.put(std::move(candsOut), "constituents");

    return true;
  }

protected:
  edm::EDGetTokenT<typename edm::View<T>> srcToken_;
  StringCutObjectSelector<T> selector_;

};

using PFJetConstituentSelector = JetConstituentSelector<reco::PFJet>;
using PatJetConstituentSelector = JetConstituentSelector<pat::Jet, std::vector<edm::FwdPtr<pat::PackedCandidate>>>;
using MiniAODJetConstituentSelector = JetConstituentSelector<reco::PFJet, std::vector<edm::FwdPtr<pat::PackedCandidate>>>;

DEFINE_FWK_MODULE(PFJetConstituentSelector);
DEFINE_FWK_MODULE(PatJetConstituentSelector);
DEFINE_FWK_MODULE(MiniAODJetConstituentSelector);
