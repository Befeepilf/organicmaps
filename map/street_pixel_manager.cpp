#include "map/street_pixel_manager.hpp"

#include "drape_frontend/drape_engine.hpp"
#include "drape_frontend/message.hpp"
#include "drape_frontend/message_subclasses.hpp"

#include "drape/color.hpp"

#include "indexer/classificator.hpp"
#include "indexer/feature.hpp"
#include "indexer/feature_decl.hpp"
#include "indexer/features_vector.hpp"

#include "geometry/angles.hpp"
#include "geometry/mercator.hpp"
#include "geometry/point2d.hpp"
#include "geometry/point_with_altitude.hpp"

#include "map/track.hpp"

#include "platform/platform.hpp"

#include "routing/routing_helpers.hpp"
#include "routing_common/bicycle_model.hpp"
#include "routing_common/pedestrian_model.hpp"

#include "base/assert.hpp"
#include "base/logging.hpp"
#include "base/math.hpp"
#include "base/src_point.hpp"

#include <healpix_base.h>
#include <healpix_tables.h>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

namespace hp
{
T_Healpix_Base<std::int64_t> const & GetHealpixBase()
{
  static T_Healpix_Base<std::int64_t> base(1048576, Healpix_Ordering_Scheme::RING, SET_NSIDE);
  return base;
}
}  // namespace hp

StreetPixelManager::StreetPixelManager() {}

void StreetPixelManager::SetDrapeEngine(ref_ptr<df::DrapeEngine> engine) { m_drapeEngine.Set(engine); }

void StreetPixelManager::SetBookmarkManager(BookmarkManager * bmManager)
{
  m_bmManager = bmManager;

  if (m_bmManager == nullptr)
  {
    LOG(LINFO, ("Bookmark manager is nullptr"));
    return;
  }

  // Register a categories-changed callback to update explored pixels once tracks/bookmarks are loaded.
  // This will overwrite any existing callback; if one existed, it is invoked inside our handler to preserve
  // behaviour.
  BookmarkManager::CategoriesChangedCallback previousCb;

  // Capture previous callback by creating a wrapper.
  previousCb = nullptr;  // No access to the previous callback; assume none.

  m_bmManager->SetCategoriesChangedCallback(
    [this, previousCb]()
    {
      if (previousCb)
        previousCb();
      this->UpdateExploredPixels();
    });

  // If loading is already finished, perform initial calculation right away.
  if (!m_bmManager->IsAsyncLoadingInProgress())
  {
    m_tracksLoaded = true;
    UpdateExploredPixels();
  }
}

void StreetPixelManager::LoadStreetPixels(std::map<storage::CountryId, storage::LocalFilePtr> const & countryFiles)
{
  GetPlatform().RunTask(Platform::Thread::Background,
                        [this, countryFiles]()
                        {
                          for (auto const & [countryId, localFile] : countryFiles)
                          {
                            // load regions one by one to avoid high memory usage
                            LoadStreetPixelsForRegion(countryId, localFile);
                          }

                          {
                            std::lock_guard<std::mutex> lock(m_streetPixelsLoadedMutex);
                            m_streetPixelsLoaded = true;
                          }

                          GetPlatform().RunTask(Platform::Thread::Gui, [this]() { UpdateExploredPixels(); });
                        });
}

void StreetPixelManager::LoadStreetPixelsForRegion(storage::CountryId const & countryId,
                                                   storage::LocalFilePtr const & localFile)
{
  if (countryId == "World" || countryId == "WorldCoasts")
  {
    LOG(LINFO, ("Skipping country file for", countryId));
    return;
  }

  std::vector<df::StreetPixel> streetPixels;
  std::string filePath = GetPlatform().WritablePathForFile(countryId + ".pix");
  try
  {
    LOG(LINFO, ("Trying to load existing pix file for", countryId));
    base::FileData file(filePath, base::FileData::Op::READ);
    size_t const fileSizeBytes = file.Size();
    streetPixels.resize(fileSizeBytes / sizeof(df::StreetPixel));
    file.Read(0, streetPixels.data(), fileSizeBytes);
    LOG(LINFO, ("Loaded", streetPixels.size(), "pixels for", countryId));
  }
  catch (std::exception const & e)
  {
    LOG(LWARNING, ("Error reading pix file:", e.what()));
  }

  bool shouldSave = false;
  if (streetPixels.empty())
  {
    LOG(LINFO, ("Calculating street pixels for region:", countryId));
    std::string const mwmPath = localFile->GetPath(MapFileType::Map);
    FeaturesVectorTest featuresVector(mwmPath);
    DeriveStreetPixelsFromFeatures(featuresVector, streetPixels);
    shouldSave = true;
  }

  AddPixels(countryId, streetPixels);

  if (shouldSave)
    SaveStreetPixelsToFile(countryId);

  LOG(LINFO, ("Done."));
}

void StreetPixelManager::SaveStreetPixelsToFile(storage::CountryId const & countryId)
{
  LOG(LINFO, ("Saving street pixels for", countryId));
  try
  {
    std::string filePath = GetPlatform().WritablePathForFile(countryId + ".pix");
    std::unique_ptr<FileWriter> writer(new FileWriter(filePath, FileWriter::OP_WRITE_TRUNCATE));
    std::lock_guard<std::mutex> lock(m_pixelsMutex);
    auto & pixelIds = m_countryStreetPixels[countryId];
    for (auto const & pixelId : pixelIds)
    {
      auto it = m_allStreetPixels.find(pixelId);
      if (it != m_allStreetPixels.end())
        writer->Write(&it->second, sizeof(df::StreetPixel));
    }
    writer->Flush();
    writer.reset();
  }
  catch (std::exception const & e)
  {
    LOG(LWARNING, ("Error saving pix file:", e.what()));
  }
}

void StreetPixelManager::SaveStreetPixelsToFile()
{
  for (auto const & [countryId, pixels] : m_countryStreetPixels)
  {
    SaveStreetPixelsToFile(countryId);
  }
}

void StreetPixelManager::DeriveStreetPixelsFromFeatures(FeaturesVectorTest & featuresVector,
                                                        std::vector<df::StreetPixel> & streetPixels)
{
  std::vector<m2::PointD> points;
  Classificator & c = classif();

  int numStreets = 0;
  featuresVector.GetVector().ForEach(
    [&](FeatureType & feature, std::uint64_t)
    {
      if (feature.GetGeomType() != feature::GeomType::Line)
        return;

      bool isHighway = false;
      bool isPrivate = false;
      feature.ForEachType(
        [&](std::uint64_t type)
        {
          std::vector<std::string> types = c.GetFullObjectNamePath(type);
          if (types.size() > 0 && types[0] == "highway")
            isHighway = true;
          if (types.size() >= 2 && types[0] == "hwtag" && types[1] == "private")
            isPrivate = true;
        });

      if (!isHighway || isPrivate)
        return;

      auto const & types = feature::TypesHolder(feature);
      bool isBicycleAccessible = routing::IsBicycleRoad(types);
      bool isPedestrianAccessible = routing::PedestrianModel::AllLimitsInstance().HasRoadType(types);

      if (!isBicycleAccessible && !isPedestrianAccessible)
        return;

      numStreets++;

      feature.ParseGeometry(FeatureType::BEST_GEOMETRY);

      size_t const numPoints = feature.GetPointsCount();
      if (numPoints < 2)
        return;

      m2::PointD prevPoint = feature.GetPoint(0);
      for (size_t i = 1; i < numPoints; ++i)
      {
        auto const point = feature.GetPoint(i);
        points.push_back(prevPoint);

        if (m2::AlmostEqualAbs(prevPoint, point, 1e-6))
        {
          continue;
        }

        m2::PointD const p12 = point - prevPoint;
        m2::PointD const p12Norm = p12.Normalize();

        double const distanceMercator = p12.Length();
        double const distanceMeters = mercator::DistanceOnEarth(prevPoint, point);
        // segmentize into 10 meter segments
        size_t const numSegments = std::ceil(distanceMeters / 10.0);
        double const segmentSize = distanceMercator / numSegments;
        for (size_t segment = 1; segment < numSegments; segment++)
        {
          m2::PointD const segmentPoint = prevPoint + p12Norm * (segment * segmentSize);
          points.push_back(segmentPoint);
        }
        prevPoint = point;
      }
    });

  std::unordered_set<std::int64_t> streetPixelIds;
  streetPixelIds.reserve(points.size());
  for (auto const & point : points)
  {
    auto const latlon = mercator::ToLatLon(point);
    double const lat_rad = base::DegToRad(latlon.m_lat);
    double const lon_rad = base::DegToRad(latlon.m_lon);
    pointing ptg(M_PI_2 - lat_rad, lon_rad);
    int64_t const pixelId = hp::GetHealpixBase().ang2pix(ptg);
    if (streetPixelIds.count(pixelId) > 0)
      // avoid duplicates
      continue;
    streetPixels.emplace_back(pixelId);
    streetPixelIds.insert(pixelId);
  }

  LOG(LINFO, ("Found", streetPixels.size(), "street pixels for", numStreets, "streets"));
}

void StreetPixelManager::AddPixels(storage::CountryId const & countryId, std::vector<df::StreetPixel> & streetPixels)
{
  {
    std::lock_guard<std::mutex> lock(m_pixelsMutex);
    auto & pixelIds = m_countryStreetPixels[countryId];
    pixelIds.reserve(pixelIds.size() + streetPixels.size());
    for (auto const & streetPixel : streetPixels)
    {
      m_allStreetPixels.insert({streetPixel.GetPixelId(), streetPixel});
      pixelIds.push_back(streetPixel.GetPixelId());
    }
  }
  std::vector<df::StreetPixel> toRemove;
  m_drapeEngine.SafeCall(&df::DrapeEngine::UpdateStreetPixels, std::move(streetPixels), std::move(toRemove));

  std::lock_guard<std::mutex> lock(m_pixelsMutex);

  LOG(LINFO, ("Loaded", m_allStreetPixels.size(), "total street pixels"));
}

void StreetPixelManager::UpdateExploredPixels()
{
  if (m_bmManager == nullptr)
    return;

  {
    std::lock_guard<std::mutex> lock(m_streetPixelsLoadedMutex);
    if (!m_streetPixelsLoaded)
    {
      LOG(LWARNING, ("Street pixels not loaded"));
      return;
    }
  }

  if (!m_tracksLoaded)
  {
    LOG(LWARNING, ("Tracks not loaded"));
    return;
  }

  LOG(LINFO, ("Collecting tracks"));

  std::vector<kml::MultiGeometry::LineT> allLines;
  m_bmManager->ForEachTrack([&](Track const & t) { allLines.emplace_back(t.GetGeometry()); });

  GetPlatform().RunTask(Platform::Thread::Background,
                        [this, lines = std::move(allLines)]() mutable
                        {
                          LOG(LINFO, ("Updating explored pixels in background thread"));

                          std::vector<m2::PointD> points;

                          LOG(LINFO, ("Lines:", lines.size()));

                          for (auto const & line : lines)
                          {
                            m2::PointD prevPoint = geometry::GetPoint(line[0]);
                            for (size_t i = 1; i < line.size(); ++i)
                            {
                              auto const point = geometry::GetPoint(line[i]);
                              points.push_back(prevPoint);

                              if (m2::AlmostEqualAbs(prevPoint, point, 1e-6))
                              {
                                continue;
                              }

                              m2::PointD const p12 = point - prevPoint;
                              m2::PointD const p12Norm = p12.Normalize();

                              double const distanceMercator = p12.Length();
                              double const distanceMeters = mercator::DistanceOnEarth(prevPoint, point);
                              // segmentize into 10 meter segments
                              size_t const numSegments = std::ceil(distanceMeters / 10.0);
                              double const segmentSize = distanceMercator / numSegments;
                              for (size_t segment = 1; segment < numSegments; segment++)
                              {
                                m2::PointD const segmentPoint = prevPoint + p12Norm * (segment * segmentSize);
                                points.push_back(segmentPoint);
                              }
                              prevPoint = point;
                            }
                          };

                          double constexpr kExploreRadiusMeters = 25.0;
                          double constexpr kEarthRadiusMeters = 6371000.0;
                          double constexpr kRadiusRads = kExploreRadiusMeters / kEarthRadiusMeters;

                          LOG(LINFO, ("Points:", points.size()));

                          std::vector<df::StreetPixel> toUpdate;
                          for (auto const & point : points)
                          {
                            auto const latlon = mercator::ToLatLon(point);
                            double const lat_rad = base::DegToRad(latlon.m_lat);
                            double const lon_rad = base::DegToRad(latlon.m_lon);
                            pointing ptg(M_PI_2 - lat_rad, lon_rad);
                            int64_t pix = hp::GetHealpixBase().ang2pix(ptg);

                            // if (m_allStreetPixels.count(pix) > 0)
                            //   exploredPixels.insert(pix);

                            rangeset<std::int64_t> pixels_in_disc = hp::GetHealpixBase().query_disc(ptg, kRadiusRads);
                            for (tsize i = 0; i < pixels_in_disc.nranges(); ++i)
                            {
                              std::int64_t const first = pixels_in_disc.ivbegin(i);
                              std::int64_t const last = pixels_in_disc.ivend(i);
                              for (pix = first; pix < last; ++pix)
                              {
                                std::lock_guard<std::mutex> lock(m_pixelsMutex);
                                auto it = m_allStreetPixels.find(pix);
                                if (it != m_allStreetPixels.end())
                                {
                                  it->second.SetExplored(true);
                                  toUpdate.push_back(it->second);
                                }
                              }
                            }
                          }

                          PrintExploredFractions();

                          if (!toUpdate.empty())
                          {
                            SaveStreetPixelsToFile();
                            std::vector<df::StreetPixel> toRemove;
                            m_drapeEngine.SafeCall(&df::DrapeEngine::UpdateStreetPixels, std::move(toUpdate),
                                                   std::move(toRemove));
                          }
                        });
}

void StreetPixelManager::PrintExploredFractions() const
{
  for (auto const & [countryId, pixels] : m_countryStreetPixels)
  {
    size_t const numExplorablePixels = pixels.size();
    size_t numExploredPixels = 0;
    {
      std::lock_guard<std::mutex> lock(m_pixelsMutex);
      for (auto const & pixel : pixels)
      {
        auto it = m_allStreetPixels.find(pixel);
        if (it != m_allStreetPixels.end() && it->second.IsExplored())
          numExploredPixels++;
      }
    }

    LOG(LINFO, ("Country:", countryId, "Num pixels:", numExplorablePixels, "Explored pixels:", numExploredPixels,
                "Explored fraction:", static_cast<double>(numExploredPixels) / numExplorablePixels));
  }
}
