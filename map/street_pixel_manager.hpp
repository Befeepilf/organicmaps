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

  void DeriveStreetPixelsFromFeatures(FeaturesVectorTest & featuresVector,
                                      std::vector<df::StreetPixelPoint> & streetPixels);

  void AddPixels(storage::CountryId const & countryId, std::vector<df::StreetPixelPoint> & streetPixels);

  void SaveStreetPixelsToFile();

  void UpdateExploredPixels();

  void PrintExploredFractions() const;

private:
  df::DrapeEngineSafePtr m_drapeEngine;

  BookmarkManager * m_bmManager = nullptr;

  T_Healpix_Base<std::int64_t> m_healpixBase;

  mutable std::mutex m_pixelsMutex;

  std::unordered_map<storage::CountryId, std::vector<std::int64_t>> m_countryStreetPixels;
  std::unordered_map<std::int64_t, df::StreetPixelPoint> m_allStreetPixels;
  std::unordered_set<std::int64_t> m_exploredPixels;
  bool m_streetPixelsLoaded = false;
  bool m_tracksLoaded = false;
};
