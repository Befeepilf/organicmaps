#pragma once

#include "map/bookmark_manager.hpp"

#include "drape_frontend/drape_engine_safe_ptr.hpp"
#include "drape_frontend/street_pixel_point.hpp"

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

class StreetPixelManager
{
public:
  StreetPixelManager();

  void SetDrapeEngine(ref_ptr<df::DrapeEngine> engine);

  void SetBookmarkManager(BookmarkManager * bmManager);

  void LoadStreetPixels(std::map<storage::CountryId, storage::LocalFilePtr> const & countryFiles);

  void LoadStreetPixelsForRegion(storage::CountryId const & countryId, storage::LocalFilePtr const & localFile);

  void DeriveStreetPixelsFromFeatures(FeaturesVectorTest & featuresVector, std::vector<std::int64_t> & streetPixels);

  void OnViewportChanged(m2::RectD const & rect);

  void AddPixels(std::vector<std::int64_t> const & streetPixels);

  void UpdateExploredPixels();
  double GetExploredFraction() const;

private:
  void UpdateViewportTask();

  df::DrapeEngineSafePtr m_drapeEngine;

  BookmarkManager * m_bmManager = nullptr;

  T_Healpix_Base<std::int64_t> m_healpixBase;

  std::unordered_set<int64_t> m_currentPixels;
  mutable std::mutex m_pixelsMutex;

  std::unordered_set<std::int64_t> m_allStreetPixels;
  std::unordered_set<std::int64_t> m_exploredPixels;
  bool m_streetPixelsLoaded = false;
  bool m_tracksLoaded = false;

  std::mutex m_viewportMutex;
  m2::RectD m_viewportRect;
  std::chrono::steady_clock::time_point m_viewportUpdatedAt;
  std::atomic<bool> m_viewportUpdateInProgress{false};
};
