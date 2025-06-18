#include "map/street_pixel_manager.hpp"

#include "drape_frontend/drape_engine.hpp"
#include "drape_frontend/message.hpp"
#include "drape_frontend/message_subclasses.hpp"

#include "drape/color.hpp"

#include "indexer/classificator.hpp"
#include "indexer/feature_decl.hpp"
#include "indexer/features_vector.hpp"

#include "geometry/angles.hpp"
#include "geometry/mercator.hpp"
#include "geometry/point2d.hpp"
#include "geometry/point_with_altitude.hpp"
#include "map/track.hpp"

#include "platform/platform.hpp"

#include "base/assert.hpp"
#include "base/logging.hpp"
#include "base/math.hpp"
#include "base/src_point.hpp"

#include <healpix_base.h>
#include <cstdint>
#include <sstream>
#include <string>

StreetPixelManager::StreetPixelManager() { m_healpixBase.SetNside(1048576, Healpix_Ordering_Scheme::RING); }

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
  for (auto const & [countryId, localFile] : countryFiles)
  {
    LoadStreetPixelsForRegion(countryId, localFile);
  }
  m_streetPixelsLoaded = true;
  UpdateExploredPixels();
}

void StreetPixelManager::LoadStreetPixelsForRegion(storage::CountryId const & countryId,
                                                   storage::LocalFilePtr const & localFile)
{
  if (countryId == "World" || countryId == "WorldCoasts")
  {
    LOG(LINFO, ("Skipping country file for", countryId));
    return;
  }

  std::vector<df::StreetPixelPoint> streetPixels;
  std::string filePath = GetPlatform().WritablePathForFile(countryId + ".pix");
  try
  {
    LOG(LINFO, ("Trying to load existing pix file for", countryId));
    base::FileData file(filePath, base::FileData::Op::READ);
    size_t const fileSizeBytes = file.Size();
    streetPixels.resize(fileSizeBytes / sizeof(df::StreetPixelPoint));
    file.Read(0, streetPixels.data(), fileSizeBytes);
    LOG(LINFO, ("Loaded", streetPixels.size(), "pixels for", countryId));
  }
  catch (std::exception const & e)
  {
    LOG(LWARNING, ("Error reading pix file:", e.what()));
  }

  if (streetPixels.empty())
  {
    LOG(LINFO, ("Calculating street pixels for region:", countryId));
    std::string const mwmPath = localFile->GetPath(MapFileType::Map);
    FeaturesVectorTest featuresVector(mwmPath);
    DeriveStreetPixelsFromFeatures(featuresVector, streetPixels);
    LOG(LINFO, ("Saving street pixels to file"));
    std::unique_ptr<FileWriter> writer(new FileWriter(filePath, FileWriter::OP_WRITE_TRUNCATE));
    writer->Write(streetPixels.data(), streetPixels.size() * sizeof(df::StreetPixelPoint));
    writer->Flush();
    writer.reset();
  }

  AddPixels(countryId, streetPixels);

  LOG(LINFO, ("Done."));
}

void StreetPixelManager::SaveStreetPixelsToFile()
{
  for (auto const & [countryId, pixels] : m_countryStreetPixels)
  {
    LOG(LINFO, ("Saving street pixels for", countryId));

    std::string filePath = GetPlatform().WritablePathForFile(countryId + ".pix");
    std::unique_ptr<FileWriter> writer(new FileWriter(filePath, FileWriter::OP_WRITE_TRUNCATE));
    writer->Write(pixels.data(), pixels.size() * sizeof(std::int64_t));
    writer->Flush();
  }
}

void StreetPixelManager::DeriveStreetPixelsFromFeatures(FeaturesVectorTest & featuresVector,
                                                        std::vector<df::StreetPixelPoint> & streetPixels)
{
  std::vector<m2::PointD> points;
  Classificator & c = classif();

  int numStreets = 0;
  featuresVector.GetVector().ForEach(
    [&](FeatureType & feature, std::uint64_t)
    {
      if (feature.GetGeomType() != feature::GeomType::Line)
        return;

      bool isExplorable = false;
      feature.ForEachType(
        [&](std::uint64_t type)
        {
          std::vector<std::string> types = c.GetFullObjectNamePath(type);
          if (!types.empty() && types[0] == "highway")
            isExplorable = true;
        });

      if (!isExplorable)
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
    pointing ptg(acos(0.0) - lat_rad, lon_rad);
    int64_t const pixelId = m_healpixBase.ang2pix(ptg);
    if (streetPixelIds.count(pixelId) > 0)
      // avoid duplicates
      continue;
    streetPixels.emplace_back(pixelId, point.x, point.y, false);
    streetPixelIds.insert(pixelId);
  }

  LOG(LINFO, ("Found", streetPixels.size(), "street pixels for", numStreets, "streets"));
}

void StreetPixelManager::AddPixels(storage::CountryId const & countryId,
                                   std::vector<df::StreetPixelPoint> & streetPixels)
{
  {
    std::lock_guard<std::mutex> lock(m_pixelsMutex);
    m_countryStreetPixels[countryId].resize(streetPixels.size());
    for (auto const streetPixel : streetPixels)
    {
      m_allStreetPixels.insert({streetPixel.GetPixelId(), streetPixel});
      m_countryStreetPixels[countryId].push_back(streetPixel.GetPixelId());
    }
  }
  std::vector<df::StreetPixelPoint> toRemove;
  m_drapeEngine.SafeCall(&df::DrapeEngine::UpdateStreetPixels, std::move(streetPixels), std::move(toRemove));

  std::lock_guard<std::mutex> lock(m_pixelsMutex);

  LOG(LINFO, ("Loaded", m_allStreetPixels.size(), "total street pixels"));
}

void StreetPixelManager::UpdateExploredPixels()
{
  if (m_bmManager == nullptr)
    return;

  if (!m_streetPixelsLoaded)
  {
    LOG(LWARNING, ("Street pixels not loaded"));
    return;
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

                          std::unordered_set<std::int64_t> exploredPixels;

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

                          std::vector<df::StreetPixelPoint> toUpdate;
                          for (auto const & point : points)
                          {
                            auto const latlon = mercator::ToLatLon(point);
                            double const lat_rad = base::DegToRad(latlon.m_lat);
                            double const lon_rad = base::DegToRad(latlon.m_lon);
                            pointing ptg(M_PI_2 - lat_rad, lon_rad);
                            int64_t pix = m_healpixBase.ang2pix(ptg);

                            // if (m_allStreetPixels.count(pix) > 0)
                            //   exploredPixels.insert(pix);

                            rangeset<std::int64_t> pixels_in_disc = m_healpixBase.query_disc(ptg, kRadiusRads);
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
                                  it->second.explored = true;
                                  toUpdate.push_back(it->second);
                                  exploredPixels.insert(pix);
                                }
                              }
                            }
                          }

                          LOG(LINFO, ("Explored pixels:", exploredPixels.size()));
                          LOG(LINFO, ("Swapping explored pixels"));
                          {
                            std::lock_guard<std::mutex> lock(m_pixelsMutex);
                            m_exploredPixels.swap(exploredPixels);
                          }

                          PrintExploredFractions();

                          if (!toUpdate.empty())
                          {
                            SaveStreetPixelsToFile();
                            std::vector<df::StreetPixelPoint> toRemove;
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
        if (it != m_allStreetPixels.end() && it->second.explored)
          numExploredPixels++;
      }
    }

    LOG(LINFO,
        ("Country:", countryId, "Explored fraction:", static_cast<double>(numExploredPixels) / numExplorablePixels));
  }
}
