#pragma once

#include "map/bookmark_manager.hpp"

#include "drape_frontend/drape_engine_safe_ptr.hpp"
#include "drape_frontend/street_pixel.hpp"

#include "drape/color.hpp"

#include "geometry/point2d.hpp"
#include "geometry/rect2d.hpp"

#include "indexer/features_vector.hpp"

#include "storage/storage.hpp"

#include <healpix_base.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <unordered_set>
#include <vector>

namespace hp
{
T_Healpix_Base<std::int64_t> const & GetHealpixBase();
}  // namespace hp

class StreetPixelsManager
{
public:
  enum class StreetPixelsStatus
  {
    NotReady,
    Loading,
    Ready,
  };

  struct StreetPixelsState
  {
    bool enabled = false;
    StreetPixelsStatus status = StreetPixelsStatus::NotReady;
  };

  using StreetPixelsStateChangedFn = std::function<void(bool enabled, StreetPixelsStatus status)>;

  StreetPixelsManager();

  StreetPixelsState GetState() const;
  void SetStateListener(StreetPixelsStateChangedFn const & onStateChangedFn);

  void SetEnabled(bool enabled);
  bool IsEnabled() const;

  void SetDrapeEngine(ref_ptr<df::DrapeEngine> engine);

  void SetBookmarkManager(BookmarkManager * bmManager);

  void LoadStreetPixels(std::map<storage::CountryId, storage::LocalFilePtr> const & countryFiles);

  void LoadStreetPixelsForRegion(storage::CountryId const & countryId, storage::LocalFilePtr const & localFile);

  void DeriveStreetPixelsFromFeatures(FeaturesVectorTest & featuresVector, std::vector<df::StreetPixel> & streetPixels);

  void AddPixels(storage::CountryId const & countryId, std::vector<df::StreetPixel> & streetPixels);

  void SaveStreetPixelsToFile();

  void SaveStreetPixelsToFile(storage::CountryId const & countryId);

  void UpdateExploredPixels();

  void PrintExploredFractions() const;

private:
  StreetPixelsState m_state;
  StreetPixelsStateChangedFn m_onStateChangedFn;

  void ChangeState(StreetPixelsState newState);

  df::DrapeEngineSafePtr m_drapeEngine;

  BookmarkManager * m_bmManager = nullptr;

  mutable std::mutex m_pixelsMutex;

  std::unordered_map<storage::CountryId, std::vector<std::int64_t>> m_countryStreetPixels;
  std::unordered_map<std::int64_t, df::StreetPixel> m_allStreetPixels;

  mutable std::mutex m_streetPixelsLoadedMutex;
  bool m_streetPixelsLoaded = false;
  bool m_tracksLoaded = false;
};
