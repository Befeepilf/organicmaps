#pragma once

#include "map/bookmark_manager.hpp"

#include "drape_frontend/drape_engine_safe_ptr.hpp"
#include "drape_frontend/street_pixel.hpp"

#include "drape/color.hpp"

#include "geometry/point2d.hpp"
#include "geometry/rect2d.hpp"

#include "coding/mmap_reader.hpp"
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
#include <span>
#include <unordered_map>
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

  using StreetPixelsStateChangedFn =
    std::function<void(bool enabled, StreetPixelsStatus status, std::string countryId)>;

  StreetPixelsManager();

  StreetPixelsState GetState() const;
  void SetStateListener(StreetPixelsStateChangedFn const & onStateChangedFn);

  void SetEnabled(bool enabled);
  bool IsEnabled() const;

  void SetDrapeEngine(ref_ptr<df::DrapeEngine> engine);

  void SetBookmarkManager(BookmarkManager * bmManager);

  void OnBookmarksCreated();
  void LoadStreetPixels(storage::LocalFilePtr const & localFile);

  std::set<std::int64_t> DeriveStreetPixelsFromFeatures(FeaturesVectorTest & featuresVector);
  void LoadStreetPixelsFromFile(storage::CountryId const & countryId);
  void SaveStreetPixelsToFile(std::set<std::int64_t> const & streetPixels);

  void ClearPixels();

  void UpdateExploredPixels();

  void PrintExploredFractions() const;

  double GetExploredFraction(kml::TrackId trackId) const;

  double GetTotalExploredFraction() const;

  void OnUpdateCurrentCountry(storage::CountryId const & countryId, storage::LocalFilePtr const & localFile);

private:
  StreetPixelsState m_state;
  StreetPixelsStateChangedFn m_onStateChangedFn;
  mutable std::mutex m_stateMutex;

  void ChangeState(StreetPixelsState newState);

  storage::CountryId m_countryId;
  mutable std::mutex m_countryIdMutex;

  df::DrapeEngineSafePtr m_drapeEngine;

  BookmarkManager * m_bmManager = nullptr;

  std::span<df::StreetPixel> m_streetPixels;

  std::unique_ptr<MmapReader> m_mmapReader;

  df::StreetPixel * FindStreetPixel(std::int64_t pixelId);

  bool m_tracksLoaded = false;

  mutable std::mutex m_fractionMutex;
  std::unordered_map<kml::TrackId, double> m_trackExploredFraction;

  void LoadExploredFractions();
  void SaveExploredFractions() const;

  std::set<int64_t> ComputeTrackPixels(kml::MultiGeometry::LineT const & line) const;
};
