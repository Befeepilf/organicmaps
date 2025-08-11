#include "map/street_pixels_manager.hpp"

#include "base/assert.hpp"
#include "base/logging.hpp"
#include "base/math.hpp"
#include "base/src_point.hpp"

#include "coding/mmap_reader.hpp"

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

#include "kml/type_utils.hpp"
#include "map/track.hpp"

#include "platform/location.hpp"
#include "platform/platform.hpp"
#include "platform/vibration.hpp"

#include "routing/routing_helpers.hpp"
#include "routing_common/bicycle_model.hpp"
#include "routing_common/pedestrian_model.hpp"

#include <healpix_base.h>
#include <healpix_tables.h>
#include <sys/mman.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
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

StreetPixelsManager::StreetPixelsManager() {}

StreetPixelsManager::StreetPixelsState StreetPixelsManager::GetState() const { return m_state; }

void StreetPixelsManager::SetStateListener(StreetPixelsStateChangedFn const & onStateChangedFn)
{
  m_onStateChangedFn = onStateChangedFn;
}

void StreetPixelsManager::ChangeState(StreetPixelsState newState)
{
  if (m_state.enabled == newState.enabled && m_state.status == newState.status)
    return;

  LOG(LINFO, ("Setting status. Is loading:", newState.status == StreetPixelsStatus::Loading));

  m_state = newState;
  if (m_onStateChangedFn != nullptr)
  {
    GetPlatform().RunTask(Platform::Thread::Gui,
                          [this]()
                          {
                            std::lock_guard<std::mutex> lock(m_countryIdMutex);
                            m_onStateChangedFn(m_state.enabled, m_state.status, m_countryId);
                          });
  }
}

void StreetPixelsManager::SetEnabled(bool enabled)
{
  ChangeState(StreetPixelsState{enabled, m_state.status});
  m_drapeEngine.SafeCall(&df::DrapeEngine::EnableStreetPixels, enabled);
}

bool StreetPixelsManager::IsEnabled() const { return m_state.enabled; }

void StreetPixelsManager::SetDrapeEngine(ref_ptr<df::DrapeEngine> engine) { m_drapeEngine.Set(engine); }

void StreetPixelsManager::SetBookmarkManager(BookmarkManager * bmManager) { m_bmManager = bmManager; }

void StreetPixelsManager::OnBookmarksCreated()
{
  m_tracksLoaded = true;
  UpdateExploredPixels();
}

void StreetPixelsManager::SetExplorationListener(ExplorationListener const & listener)
{
  m_explorationListener = listener;
}

void StreetPixelsManager::LoadStreetPixels(storage::LocalFilePtr const & localFile)
{
  storage::CountryId countryId;
  {
    std::lock_guard<std::mutex> lock(m_countryIdMutex);
    countryId = m_countryId;
  }

  if (countryId == "World" || countryId == "WorldCoasts")
  {
    LOG(LINFO, ("Skipping country file for", countryId));
    return;
  }

  try
  {
    LoadStreetPixelsFromFile(countryId);
  }
  catch (std::exception const & e)
  {
    LOG(LERROR, ("Failed to memory-map pix file:", e.what()));

    LOG(LINFO, ("Calculating street pixels for region:", countryId));
    std::string const mwmPath = localFile->GetPath(MapFileType::Map);
    FeaturesVectorTest featuresVector(mwmPath);
    auto newStreetPixels = DeriveStreetPixelsFromFeatures(featuresVector);
    SaveStreetPixelsToFile(newStreetPixels);
    LoadStreetPixelsFromFile(countryId);
  }

  {
    std::lock_guard<std::mutex> lock(m_countryIdMutex);
    if (m_countryId != countryId)
    {
      LOG(LWARNING, ("Country changed while loading street pixels. Aborting."));
      return;
    }
  }

  m_drapeEngine.SafeCall(&df::DrapeEngine::UpdateStreetPixels, m_streetPixels);

  LOG(LINFO, ("Loaded", m_streetPixels.size(), "total street pixels"));
}

void StreetPixelsManager::LoadStreetPixelsFromFile(storage::CountryId const & countryId)
{
  std::string filePath = GetPlatform().WritablePathForFile(countryId + ".pix");
  LOG(LINFO, ("Trying to memory-map existing pix file for", countryId));
  m_mmapReader = std::make_unique<MmapReader>(filePath, MmapReader::Advice::Sequential, true);
  m_streetPixels = m_mmapReader->DataSpan<df::StreetPixel>();
  LOG(LINFO, ("Mapped", m_streetPixels.size(), "pixels for", countryId));

  LOG(LINFO, ("first ids in file:", m_streetPixels[0].GetPixelId(), m_streetPixels[1].GetPixelId(),
              m_streetPixels[2].GetPixelId(), "Last id:", m_streetPixels[m_streetPixels.size() - 1].GetPixelId()));
}

void StreetPixelsManager::SaveStreetPixelsToFile(std::set<std::int64_t> const & streetPixels)
{
  storage::CountryId countryId;
  {
    std::lock_guard<std::mutex> lock(m_countryIdMutex);
    countryId = m_countryId;
  }

  LOG(LINFO, ("Saving street pixels for", countryId));
  std::string filePath = GetPlatform().WritablePathForFile(countryId + ".pix");
  std::unique_ptr<FileWriter> writer(new FileWriter(filePath, FileWriter::OP_WRITE_TRUNCATE));
  for (auto const & pixel : streetPixels)
    writer->Write(&pixel, sizeof(int64_t));
  writer->Flush();
  writer.reset();
}

std::set<std::int64_t> StreetPixelsManager::DeriveStreetPixelsFromFeatures(FeaturesVectorTest & featuresVector)
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
      bool isBikeAccessible = true;
      bool isPedestrianAccessible = true;
      feature.ForEachType(
        [&](std::uint64_t type)
        {
          std::vector<std::string> types = c.GetFullObjectNamePath(type);
          if (types.size() > 0 && types[0] == "highway")
          {
            if (types.size() < 3 || (types[2] != "driveway" && types[2] != "tunnel"))
              isHighway = true;
          }
          if (types.size() >= 2 && types[0] == "hwtag")
          {
            if (types[1] == "private")
              isPrivate = true;
            else if (types[1] == "nobicycle")
              isBikeAccessible = false;
            else if (types[1] == "yesbicycle")
              isBikeAccessible = true;
            else if (types[1] == "nofoot")
              isPedestrianAccessible = false;
            else if (types[1] == "yesfoot")
              isPedestrianAccessible = true;
          }
        });

      if (!isHighway || isPrivate || (!isBikeAccessible && !isPedestrianAccessible))
        return;

      // auto const & types = feature::TypesHolder(feature);
      // bool isBicycleAccessible = routing::IsBicycleRoad(types);
      // bool isPedestrianAccessible = routing::PedestrianModel::AllLimitsInstance().HasRoadType(types);

      // if (!isBicycleAccessible && !isPedestrianAccessible)
      //   return;

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
        // segmentize into 15 meter segments
        size_t const numSegments = std::ceil(distanceMeters / 15.0);
        double const segmentSize = distanceMercator / numSegments;
        for (size_t segment = 1; segment < numSegments; segment++)
        {
          m2::PointD const segmentPoint = prevPoint + p12Norm * (segment * segmentSize);
          points.push_back(segmentPoint);
        }
        prevPoint = point;
      }
    });

  std::set<std::int64_t> streetPixels;
  for (auto const & point : points)
  {
    auto const latlon = mercator::ToLatLon(point);
    double const lat_rad = base::DegToRad(latlon.m_lat);
    double const lon_rad = base::DegToRad(latlon.m_lon);
    pointing ptg(M_PI_2 - lat_rad, lon_rad);
    std::int64_t const pixelId = hp::GetHealpixBase().ang2pix(ptg);
    if (streetPixels.count(pixelId) > 0)
      // avoid duplicates
      continue;
    streetPixels.insert(pixelId);
  }

  LOG(LINFO, ("Found", streetPixels.size(), "street pixels for", numStreets, "streets"));
  return streetPixels;
}

df::StreetPixel * StreetPixelsManager::FindStreetPixel(std::int64_t pixelId)
{
  auto first = m_streetPixels.begin();
  auto last = m_streetPixels.end();
  auto it = std::lower_bound(first, last, pixelId,
                             [](df::StreetPixel const & p, std::int64_t id) { return p.GetPixelId() < id; });
  if (it != last && it->GetPixelId() == pixelId)
    return &(*it);
  return nullptr;
}

void StreetPixelsManager::UpdateExploredPixels()
{
  if (m_bmManager == nullptr)
    return;

  {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    if (m_state.status != StreetPixelsStatus::Ready)
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
  struct TrackInfo
  {
    kml::TrackId id;
    kml::MultiGeometry::LineT geom;
    kml::Timestamp ts;
  };
  std::vector<TrackInfo> tracks;
  std::unordered_map<kml::TrackId, double> trackExploredFraction;
  m_bmManager->ForEachTrackSortedByTimestamp(
    [&](Track const & t) { tracks.push_back(TrackInfo{t.GetId(), t.GetGeometry(), t.GetData().m_timestamp}); });

  storage::CountryId countryId;
  {
    std::lock_guard<std::mutex> lock(m_countryIdMutex);
    countryId = m_countryId;
  }

  GetPlatform().RunTask(
    Platform::Thread::Background,
    [this, tracks = std::move(tracks), trackExploredFraction, countryId]() mutable
    {
      for (auto const & ti : tracks)
      {
        {
          std::lock_guard<std::mutex> lock(m_countryIdMutex);
          if (m_countryId != countryId)
          {
            LOG(LWARNING, ("Country changed while updating explored pixels. Aborting."));
            return;
          }
        }

        if (HasExploredFraction(ti.id))
          continue;

        LOG(LINFO, ("Computing track pixels for", ti.id));

        auto trackPixels = ComputeTrackPixels(ti.geom);
        std::set<int64_t> newPixels;
        {
          for (auto pix : trackPixels)
          {
            auto * pixel = FindStreetPixel(pix);
            if (pixel == nullptr || pixel->IsExplored())
              continue;
            pixel->SetExplored(true);
            msync(pixel, sizeof(df::StreetPixel), MS_ASYNC);
            newPixels.insert(pix);
          }
        }

        trackExploredFraction[ti.id] =
          trackPixels.empty() ? 0.0
                              : static_cast<double>(newPixels.size()) / static_cast<double>(m_streetPixels.size());

        LOG(LINFO, ("Track", ti.id, "explored fraction:", trackExploredFraction[ti.id]));

        if (!m_explorationListener)
        {
          LOG(LWARNING, ("No exploration listener"));
        }
        if (newPixels.empty())
        {
          LOG(LWARNING, ("No new pixels"));
        }

        if (!newPixels.empty() && m_explorationListener)
        {
          ExplorationDelta d;
          d.m_regionId = countryId;
          d.m_newPixels = static_cast<uint32_t>(newPixels.size());
          d.m_eventTimeSec = static_cast<double>(kml::ToSecondsSinceEpoch(ti.ts));
          m_explorationListener(d);
        }
      }

      {
        std::lock_guard<std::mutex> lock(m_countryIdMutex);
        if (m_countryId != countryId)
        {
          LOG(LWARNING, ("Country changed while updating explored pixels. Aborting."));
          return;
        }
      }

      {
        std::lock_guard<std::mutex> lock(m_fractionMutex);
        m_trackExploredFraction = std::move(trackExploredFraction);
      }

      LOG(LINFO, ("Calculated explored fractions"));

      SaveExploredFractions();

      // Notify UI that exploration data updated even if status unchanged.
      if (m_onStateChangedFn)
      {
        GetPlatform().RunTask(Platform::Thread::Gui,
                              [this]()
                              {
                                std::lock_guard<std::mutex> lock(m_countryIdMutex);
                                m_onStateChangedFn(m_state.enabled, m_state.status, m_countryId);
                              });
      }
    });
}

std::set<int64_t> StreetPixelsManager::ComputeTrackPixels(kml::MultiGeometry::LineT const & line) const
{
  std::set<int64_t> pixels;

  if (line.empty())
    return pixels;

  m2::PointD prev = geometry::GetPoint(line[0]);
  for (size_t i = 1; i < line.size(); ++i)
  {
    auto const & ptWithAlt = line[i];
    m2::PointD curr = geometry::GetPoint(ptWithAlt);
    double distMerc = (curr - prev).Length();
    double distMeters = mercator::DistanceOnEarth(prev, curr);
    size_t segments = std::max<size_t>(1, static_cast<size_t>(std::ceil(distMeters / 10.0)));
    m2::PointD dir = (curr - prev).Normalize();
    double step = distMerc / segments;
    for (size_t s = 0; s <= segments; ++s)
    {
      m2::PointD p = prev + dir * (s * step);
      auto const latlon = mercator::ToLatLon(p);
      AddPixelsInRadius(latlon.m_lat, latlon.m_lon, pixels);
    }
    prev = curr;
  }
  return pixels;
}

void StreetPixelsManager::AddPixelsInRadius(double lat, double lon, std::set<std::int64_t> & pixels) const
{
  double constexpr kExploreRadiusMeters = 20.0;
  double constexpr kEarthRadiusMeters = 6371000.0;
  double constexpr kRadiusRads = kExploreRadiusMeters / kEarthRadiusMeters;

  double const lat_rad = base::DegToRad(lat);
  double const lon_rad = base::DegToRad(lon);
  pointing ang(M_PI_2 - lat_rad, lon_rad);
  auto disc = hp::GetHealpixBase().query_disc(ang, kRadiusRads);
  for (tsize r = 0; r < disc.nranges(); ++r)
  {
    std::int64_t first = disc.ivbegin(r);
    std::int64_t last = disc.ivend(r);
    for (std::int64_t pix = first; pix < last; ++pix)
      pixels.insert(pix);
  }
}

void StreetPixelsManager::OnLocationUpdate(location::GpsInfo const & info)
{
  auto const latlon = info.GetLatLon();
  std::set<std::int64_t> pixels;
  AddPixelsInRadius(latlon.m_lat, latlon.m_lon, pixels);
  size_t numNewlyExploredPixels = 0;
  for (auto const & pix : pixels)
  {
    auto * pixel = FindStreetPixel(pix);
    if (pixel == nullptr || pixel->IsExplored())
      continue;
    pixel->SetExplored(true);
    msync(pixel, sizeof(df::StreetPixel), MS_ASYNC);
    numNewlyExploredPixels++;
  }

  if (numNewlyExploredPixels > 0 && m_explorationListener)
  {
    ExplorationDelta d;
    {
      std::lock_guard<std::mutex> lock(m_countryIdMutex);
      d.m_regionId = m_countryId;
    }
    d.m_newPixels = static_cast<uint32_t>(numNewlyExploredPixels);
    d.m_eventTimeSec = info.m_timestamp;
    m_explorationListener(d);
  }

  if (numNewlyExploredPixels == 1)
    platform::Vibrate(50);
  else if (numNewlyExploredPixels > 1)
  {
    size_t const maxPixels = 5;  // Limit to avoid too long vibration
    size_t const count = std::min(numNewlyExploredPixels, maxPixels);

    std::vector<uint32_t> durations(count, 30);
    std::vector<uint32_t> delays(count, 20);

    platform::VibratePattern(durations.data(), delays.data(), count);
  }
}

std::string StreetPixelsManager::GetCurrentCountryId() const
{
  std::lock_guard<std::mutex> lock(m_countryIdMutex);
  return m_countryId;
}

void StreetPixelsManager::OnUpdateCurrentCountry(storage::CountryId const & countryId,
                                                 storage::LocalFilePtr const & localFile)
{
  {
    std::lock_guard<std::mutex> lock(m_countryIdMutex);
    LOG(LINFO, ("Country changed from", m_countryId, "to", countryId));
    if (countryId == m_countryId)
      return;
    m_countryId = countryId;
  }

  ClearPixels();
  if (countryId.empty())
    return;

  if (!localFile || !localFile->OnDisk(MapFileType::Map))
    return;

  LoadExploredFractions();
  ChangeState(StreetPixelsState{m_state.enabled, StreetPixelsStatus::Loading});
  GetPlatform().RunTask(Platform::Thread::Background,
                        [this, countryId, localFile]()
                        {
                          LoadStreetPixels(localFile);
                          ChangeState(StreetPixelsState{m_state.enabled, StreetPixelsStatus::Ready});
                          GetPlatform().RunTask(Platform::Thread::Gui, [this]() { UpdateExploredPixels(); });
                        });
}

bool StreetPixelsManager::HasExploredFraction(kml::TrackId const & trackId) const
{
  std::lock_guard<std::mutex> lock(m_fractionMutex);
  return m_trackExploredFraction.find(trackId) != m_trackExploredFraction.end();
}

double StreetPixelsManager::GetExploredFraction(kml::TrackId const & trackId) const
{
  std::lock_guard<std::mutex> lock(m_fractionMutex);
  auto it = m_trackExploredFraction.find(trackId);
  return it != m_trackExploredFraction.end() ? it->second : 0.0;
}

void StreetPixelsManager::LoadExploredFractions()
{
  std::lock_guard<std::mutex> lock(m_fractionMutex);
  m_trackExploredFraction.clear();
  storage::CountryId country;
  {
    std::lock_guard<std::mutex> lock(m_countryIdMutex);
    country = m_countryId;
  }
  std::string path = GetPlatform().WritablePathForFile(country + ".pixf");
  std::ifstream ifs(path);
  if (!ifs.is_open())
  {
    LOG(LINFO, ("No explored fractions file for", country));
    return;
  }
  kml::TrackId id;
  double frac;
  while (ifs >> id >> frac)
    m_trackExploredFraction[id] = frac;
}

void StreetPixelsManager::SaveExploredFractions() const
{
  LOG(LINFO, ("Saving explored fractions"));

  std::lock_guard<std::mutex> lock(m_fractionMutex);
  storage::CountryId country;
  {
    std::lock_guard<std::mutex> lock(m_countryIdMutex);
    country = m_countryId;
  }
  std::string path = GetPlatform().WritablePathForFile(country + ".pixf");
  std::ofstream ofs(path, std::ofstream::trunc);
  if (!ofs.is_open())
  {
    LOG(LWARNING, ("Failed to open explored fractions file for writing:", path));
    return;
  }
  for (auto const & kv : m_trackExploredFraction)
    ofs << kv.first << " " << kv.second << "\n";
}

double StreetPixelsManager::GetTotalExploredFraction() const
{
  size_t total = m_streetPixels.size();
  if (total == 0)
    return 0.0;
  size_t explored = 0;
  for (auto const & pixel : m_streetPixels)
    if (pixel.IsExplored())
      ++explored;
  return static_cast<double>(explored) / total;
}

void StreetPixelsManager::ClearPixels()
{
  LOG(LINFO, ("Clearing pixels and unmapping pix file"));
  m_drapeEngine.SafeCall(&df::DrapeEngine::ClearStreetPixels);
  m_streetPixels = {};
  m_mmapReader.reset();

  {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    ChangeState(StreetPixelsState{m_state.enabled, StreetPixelsStatus::NotReady});
  }
}
