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

  std::vector<std::int64_t> streetPixels;

  std::string filePath = GetPlatform().WritablePathForFile(countryId + ".pix");

  try
  {
    LOG(LINFO, ("Trying to load existing pix file for", countryId));
    base::FileData file(filePath, base::FileData::Op::READ);
    size_t const fileSizeBytes = file.Size();
    streetPixels.resize(fileSizeBytes / 8);
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

    std::unique_ptr<FileWriter> writer(new FileWriter(filePath, FileWriter::OP_WRITE_TRUNCATE));
    writer->Write(streetPixels.data(), streetPixels.size() * sizeof(std::int64_t));
    writer->Flush();
    writer.reset();
  }

  AddPixels(streetPixels);

  LOG(LINFO, ("Done."));
}

void StreetPixelManager::DeriveStreetPixelsFromFeatures(FeaturesVectorTest & featuresVector,
                                                        std::vector<std::int64_t> & streetPixels)
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

  for (auto const & point : points)
  {
    auto const latlon = mercator::ToLatLon(point);
    double const lat_rad = base::DegToRad(latlon.m_lat);
    double const lon_rad = base::DegToRad(latlon.m_lon);
    pointing ptg(acos(0.0) - lat_rad, lon_rad);
    streetPixels.emplace_back(m_healpixBase.ang2pix(ptg));
  }

  std::vector<std::int64_t>::iterator ip = std::unique(streetPixels.begin(), streetPixels.end());
  streetPixels.resize(std::distance(streetPixels.begin(), ip));  // remove duplicates
  LOG(LINFO, ("Found", streetPixels.size(), "street pixels for", numStreets, "streets"));
}

void StreetPixelManager::AddPixels(std::vector<std::int64_t> const & streetPixels)
{
  m_allStreetPixels.insert(streetPixels.begin(), streetPixels.end());
  size_t const potentialSize = m_allStreetPixels.size() * sizeof(df::StreetPixelPoint);
  LOG(LINFO, ("Loaded", m_allStreetPixels.size(), "total street pixels with potential size", potentialSize, "bytes"));
}

void StreetPixelManager::OnViewportChanged(m2::RectD const & rect)
{
  {
    std::lock_guard<std::mutex> lock(m_viewportMutex);
    m_viewportRect = rect;
    m_viewportUpdatedAt = std::chrono::steady_clock::now();
  }

  bool expected = false;
  if (m_viewportUpdateInProgress.compare_exchange_strong(expected, true))
  {
    GetPlatform().RunTask(Platform::Thread::Background, [this] { this->UpdateViewportTask(); });
  }
  else
  {
    LOG(LINFO, ("Not updating viewport pixels because update already in progress"));
  }
}

void StreetPixelManager::UpdateViewportTask()
{
  m2::RectD rect;
  std::chrono::steady_clock::time_point viewportUpdatedAt;
  {
    std::lock_guard<std::mutex> lock(m_viewportMutex);
    rect = m_viewportRect;
    viewportUpdatedAt = m_viewportUpdatedAt;
  }

  if (m_allStreetPixels.empty())
  {
    LOG(LINFO, ("Not updating viewport pixels because no pixels are loaded"));
    m_viewportUpdateInProgress = false;
    return;
  }

  LOG(LINFO, ("On viewport changed"));

  while (true)
  {
    while (std::chrono::steady_clock::now() - viewportUpdatedAt < std::chrono::seconds(1))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      {
        std::lock_guard<std::mutex> lock(m_viewportMutex);
        viewportUpdatedAt = m_viewportUpdatedAt;
      }
    }

    std::vector<pointing> corners;
    double minLat = M_PI;
    double maxLat = 0;
    double minLon = M_PI;
    double maxLon = -M_PI;
    rect.ForEachCorner(
      [&](m2::PointD const & point)
      {
        auto const latlon = mercator::ToLatLon(point);
        double const lat_rad = base::DegToRad(latlon.m_lat);
        double const lon_rad = base::DegToRad(latlon.m_lon);
        minLat = std::min(minLat, lat_rad);
        maxLat = std::max(maxLat, lat_rad);
        minLon = std::min(minLon, lon_rad);
        maxLon = std::max(maxLon, lon_rad);
        LOG(LINFO, ("Adding corner", M_PI_2 - lat_rad, lon_rad));
        corners.emplace_back(pointing(M_PI_2 - lat_rad, lon_rad));
      });

    if (maxLat - minLat > 0.0012 || maxLon - minLon > 0.0012)
    {
      LOG(LINFO, ("Too large rect:", "lat diff:", maxLat - minLat, "lon diff:", maxLon - minLon));
      std::vector<df::StreetPixelPoint> toAdd;
      std::vector<int64_t> toRemove;
      {
        std::lock_guard<std::mutex> lock(m_pixelsMutex);
        toRemove.assign(m_currentPixels.begin(), m_currentPixels.end());
        m_currentPixels.clear();
      }
      if (!toRemove.empty())
      {
        m_drapeEngine.SafeCall(&df::DrapeEngine::UpdateStreetPixels, std::move(toAdd), std::move(toRemove));
      }
      m_viewportUpdateInProgress = false;
      return;
    }

    LOG(LINFO, ("Querying pixels in rect"));
    rangeset<std::int64_t> pixelsInRectRange;
    try
    {
      m_healpixBase.query_polygon(corners, pixelsInRectRange);
    }
    catch (PlanckError const & e)
    {
      LOG(LWARNING, ("Error querying pixels in rect:", e.what()));
      m_viewportUpdateInProgress = false;
      return;
    }

    LOG(LINFO, ("Got", pixelsInRectRange.nval(), "total pixels in rect"));

    std::unordered_set<std::int64_t> visiblePixels;
    visiblePixels.reserve(pixelsInRectRange.nval());
    for (tsize i = 0; i < pixelsInRectRange.nranges(); ++i)
    {
      std::int64_t const first = pixelsInRectRange.ivbegin(i);
      std::int64_t const last = pixelsInRectRange.ivend(i);
      for (std::int64_t pix = first; pix < last; ++pix)
      {
        if (m_allStreetPixels.count(pix) > 0)
          visiblePixels.insert(pix);
      }
    }

    LOG(LINFO, ("Got", visiblePixels.size(), "street pixels in rect"));

    std::vector<df::StreetPixelPoint> toAdd;
    std::vector<int64_t> toRemove;

    {
      std::lock_guard<std::mutex> lock(m_pixelsMutex);
      for (auto const & pix : m_currentPixels)
      {
        if (visiblePixels.count(pix) == 0)
          toRemove.push_back(pix);
      }

      for (auto const & pix : visiblePixels)
      {
        if (m_currentPixels.count(pix) == 0)
        {
          df::StreetPixelPoint pt;
          pt.m_pixelId = pix;
          pointing const ptg = m_healpixBase.pix2ang(pix);
          double const lat = 90.0 - ang::RadToDeg(ptg.theta);
          double const lon = ang::RadToDeg(ptg.phi);
          pt.m_point = mercator::FromLatLon(lat, lon);
          if (m_exploredPixels.count(pix) > 0)
            pt.m_color = dp::Color(0, 255, 0, 255);
          else
            pt.m_color = dp::Color(255, 0, 0, 255);
          toAdd.push_back(pt);
        }
      }
    }

    if (!toAdd.empty() || !toRemove.empty())
    {
      {
        std::lock_guard<std::mutex> lock(m_pixelsMutex);
        for (auto const & pix : toRemove)
          m_currentPixels.erase(pix);
        for (auto const & pt : toAdd)
          m_currentPixels.insert(pt.m_pixelId);
      }
      m_drapeEngine.SafeCall(&df::DrapeEngine::UpdateStreetPixels, std::move(toAdd), std::move(toRemove));
    }

    {
      std::lock_guard<std::mutex> lock(m_viewportMutex);
      if (rect == m_viewportRect)
      {
        m_viewportUpdateInProgress = false;
        break;
      }
      rect = m_viewportRect;
    }
  }
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
                          for (auto const & point : points)
                          {
                            auto const latlon = mercator::ToLatLon(point);
                            double const lat_rad = base::DegToRad(latlon.m_lat);
                            double const lon_rad = base::DegToRad(latlon.m_lon);
                            pointing ptg(M_PI_2 - lat_rad, lon_rad);
                            int64_t pix = m_healpixBase.ang2pix(ptg);

                            if (m_allStreetPixels.count(pix) > 0)
                              exploredPixels.insert(pix);

                            rangeset<std::int64_t> pixels_in_disc = m_healpixBase.query_disc(ptg, kRadiusRads);
                            for (tsize i = 0; i < pixels_in_disc.nranges(); ++i)
                            {
                              std::int64_t const first = pixels_in_disc.ivbegin(i);
                              std::int64_t const last = pixels_in_disc.ivend(i);
                              for (pix = first; pix < last; ++pix)
                              {
                                if (m_allStreetPixels.count(pix) > 0)
                                  exploredPixels.insert(pix);
                              }
                            }
                          }

                          LOG(LINFO, ("Explored pixels:", exploredPixels.size()));
                          LOG(LINFO, ("Swapping explored pixels"));
                          m_exploredPixels.swap(exploredPixels);

                          LOG(LINFO, ("EXPLORED PCT:", GetExploredFraction()));

                          if (exploredPixels.empty())
                            return;

                          std::vector<df::StreetPixelPoint> toUpdate;
                          {
                            std::lock_guard<std::mutex> lock(m_pixelsMutex);
                            for (auto const & pix : m_currentPixels)
                            {
                              if (m_exploredPixels.count(pix) > 0)
                              {
                                df::StreetPixelPoint pt;
                                pt.m_pixelId = pix;
                                pointing const ptg = m_healpixBase.pix2ang(pix);
                                double const lat = 90.0 - ang::RadToDeg(ptg.theta);
                                double const lon = ang::RadToDeg(ptg.phi);
                                pt.m_point = mercator::FromLatLon(lat, lon);
                                pt.m_color = dp::Color(0, 255, 0, 255);
                                toUpdate.push_back(pt);
                              }
                            }
                          }
                          if (!toUpdate.empty())
                          {
                            m_drapeEngine.SafeCall(&df::DrapeEngine::UpdateStreetPixels, std::move(toUpdate),
                                                   std::vector<int64_t>());
                          }
                        });
}

double StreetPixelManager::GetExploredFraction() const
{
  if (m_allStreetPixels.empty())
    return 0.0;
  return static_cast<double>(m_exploredPixels.size()) / static_cast<double>(m_allStreetPixels.size());
}
