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

#include "platform/country_file.hpp"
#include "platform/location.hpp"
#include "platform/platform.hpp"
#include "platform/vibration.hpp"

#include "routing/routing_helpers.hpp"
#include "routing_common/bicycle_model.hpp"
#include "routing_common/pedestrian_model.hpp"

#include "map/street_stats_db.hpp"

#include "indexer/feature_algo.hpp"

#include "geometry/mercator.hpp"
#include "geometry/parametrized_segment.hpp"

#include "search/reverse_geocoder.hpp"

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
  static T_Healpix_Base<std::int64_t> base(1048576, Healpix_Ordering_Scheme::NEST, SET_NSIDE);
  return base;
}
}  // namespace hp

double constexpr kSegmentLengthMeters = 15.0;
double constexpr kExploreRadiusMeters = 20.0;
double constexpr kEarthRadiusMeters = 6371000.0;
double constexpr kRadiusRads = kExploreRadiusMeters / kEarthRadiusMeters;

StreetPixelsManager::StreetPixelsManager(DataSource const & dataSource)
  : m_dataSource(dataSource)
{}

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
  LOG(LINFO, ("OnBookmarksCreated"));
  m_tracksLoaded = true;
  UpdateExploredPixels();
}

void StreetPixelsManager::SetExplorationListener(ExplorationListener const & listener)
{
  m_explorationListener = listener;
}

void StreetPixelsManager::LoadStreetPixels(storage::LocalFilePtr const & localFile)
{
  LOG(LINFO, ("LoadStreetPixels"));

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
    LOG(LWARNING, ("Failed to memory-map pix file:", e.what()));
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
  LoadAccountedBits();
}

void StreetPixelsManager::LoadStreetPixelsFromFile(storage::CountryId const & countryId)
{
  LOG(LINFO, ("LoadStreetPixelsFromFile", countryId));

  std::string filePath = GetPlatform().WritablePathForFile(countryId + ".pix");
  LOG(LINFO, ("Trying to memory-map existing pix file for", countryId));
  m_mmapReader = std::make_unique<MmapReader>(filePath, MmapReader::Advice::Sequential, true);
  m_streetPixels = m_mmapReader->DataSpan<df::StreetPixel>();
  LOG(LINFO, ("Mapped", m_streetPixels.size(), "pixels for", countryId));
}

void StreetPixelsManager::SaveStreetPixelsToFile(std::set<std::int64_t> const & streetPixels)
{
  LOG(LINFO, ("SaveStreetPixelsToFile", streetPixels.size()));

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
  LOG(LINFO, ("DeriveStreetPixelsFromFeatures"));

  MwmSet::MwmId mwmId;
  {
    std::lock_guard<std::mutex> lock(m_countryIdMutex);
    if (m_countryId.empty())
      return {};
    mwmId = m_dataSource.GetMwmIdByCountryFile(platform::CountryFile(m_countryId));
  }

  if (!mwmId.IsAlive())
    return {};

  std::map<FeatureID, std::vector<uint32_t>> featurePixelIndices;
  std::map<FeatureID, double> featureLengths;
  std::vector<m2::PointD> points;
  int numStreets = 0;
  featuresVector.GetVector().ForEach(
    [&](FeatureType & feature, std::uint64_t)
    {
      if (!IsExplorable(feature))
        return;

      numStreets++;

      feature.ParseGeometry(FeatureType::BEST_GEOMETRY);

      size_t const numPoints = feature.GetPointsCount();
      if (numPoints < 2)
        return;

      double totalLengthMeters = 0;
      m2::PointD prevPoint = feature.GetPoint(0);
      for (size_t i = 1; i < numPoints; ++i)
      {
        auto const point = feature.GetPoint(i);
        points.push_back(prevPoint);

        SegmentizeStreet(prevPoint, point,
                         [&](m2::PointD const & segmentPoint, double distFromPrevPoint)
                         {
                           points.push_back(segmentPoint);
                           double const distanceAlongFeatureM = totalLengthMeters + distFromPrevPoint;
                           uint32_t const pixelIndex = static_cast<uint32_t>(distanceAlongFeatureM / kSegmentLengthMeters);
                           featurePixelIndices[feature.GetID()].push_back(pixelIndex);
                         });

        totalLengthMeters += mercator::DistanceOnEarth(prevPoint, point);
        prevPoint = point;
      }
      featureLengths[feature.GetID()] = totalLengthMeters;
    });

  auto & db = street_stats::StreetStatsDB::Instance();
  for (auto const & [fid, pixelIndices] : featurePixelIndices)
  {
    auto bitmask = db.GetBitmask(fid.m_mwmId, fid.m_index);
    if (!bitmask)
    {
      if (!featureLengths.count(fid))
        continue;
      size_t const numPixels = static_cast<size_t>(std::ceil(featureLengths[fid] / kSegmentLengthMeters));
      size_t const numBytes = (numPixels + 7) / 8;
      bitmask.emplace(numBytes, 0);
    }

    for (uint32_t pixelIndex : pixelIndices)
    {
      size_t const byteIndex = pixelIndex / 8;
      if (byteIndex < bitmask->size())
      {
        uint8_t const bitIndex = pixelIndex % 8;
        (*bitmask)[byteIndex] |= (1 << bitIndex);
      }
    }
    db.SaveBitmask(fid.m_mwmId, fid.m_index, *bitmask);
  }

  std::set<std::int64_t> streetPixels;
  for (auto const & point : points)
  {
    auto const latlon = mercator::ToLatLon(point);
    double const lat_rad = math::DegToRad(latlon.m_lat);
    double const lon_rad = math::DegToRad(latlon.m_lon);
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

void StreetPixelsManager::SegmentizeStreet(m2::PointD const & p1, m2::PointD const & p2,
                                           std::function<void(m2::PointD const &, double)> const & callback)
{
  if (m2::AlmostEqualAbs(p1, p2, 1e-6))
    return;

  m2::PointD const p12 = p2 - p1;
  m2::PointD const p12Norm = p12.Normalize();

  double const distanceMercator = p12.Length();
  double const distanceMeters = mercator::DistanceOnEarth(p1, p2);

  size_t const numSegments = std::ceil(distanceMeters / kSegmentLengthMeters);
  if (numSegments <= 1)
    return;

  double const segmentSizeMercator = distanceMercator / numSegments;
  for (size_t i = 1; i < numSegments; ++i)
  {
    m2::PointD const segmentPoint = p1 + p12Norm * (i * segmentSizeMercator);
    double const distFromP1 = mercator::DistanceOnEarth(p1, segmentPoint);
    callback(segmentPoint, distFromP1);
  }
}

bool StreetPixelsManager::IsExplorable(FeatureType & ft) const
{
  if (ft.GetGeomType() != feature::GeomType::Line)
    return false;

  bool isHighway = false;
  bool isPrivate = false;
  bool isBikeAccessible = true;
  bool isPedestrianAccessible = true;
  Classificator & c = classif();
  ft.ForEachType(
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

  // auto const & types = feature::TypesHolder(feature);
  // bool isBicycleAccessible = routing::IsBicycleRoad(types);
  // bool isPedestrianAccessible = routing::PedestrianModel::AllLimitsInstance().HasRoadType(types);

  // if (!isBicycleAccessible && !isPedestrianAccessible)
  //   return;

  return isHighway && !isPrivate && (isBikeAccessible || isPedestrianAccessible);
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
  LOG(LINFO, ("UpdateExploredPixels"));

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

        UpdateStreetStatsForTrack(ti.geom);

        LOG(LINFO, ("Computing track pixels for", ti.id));

        auto trackPixels = ComputeTrackPixels(ti.geom);
        size_t statsNew = 0;
        std::set<int64_t> renderNew;
        for (auto pix : trackPixels)
        {
          auto * pixel = FindStreetPixel(pix);
          if (pixel == nullptr)
            continue;
          if (!pixel->IsExplored())
          {
            pixel->SetExplored(true);
            msync(pixel, sizeof(df::StreetPixel), MS_ASYNC);
            renderNew.insert(pix);
          }
          if (!m_accountedBits.empty())
          {
            size_t idx = GetPixelIndex(pixel);
            if (!IsAccountedIndex(idx))
            {
              SetAccountedIndex(idx);
              ++statsNew;
            }
          }
        }

        trackExploredFraction[ti.id] =
          trackPixels.empty() ? 0.0
                              : static_cast<double>(renderNew.size()) / static_cast<double>(m_streetPixels.size());

        LOG(LINFO, ("Track", ti.id, "explored fraction:", trackExploredFraction[ti.id]));

        if (statsNew > 0 && m_explorationListener)
        {
          ExplorationDelta d;
          d.m_regionId = countryId;
          d.m_newPixels = static_cast<uint32_t>(statsNew);
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
      if (m_accountedDirty)
        SaveAccountedBits();

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

void StreetPixelsManager::UpdateStreetStatsForTrack(kml::MultiGeometry::LineT const & line)
{
  LOG(LINFO, ("UpdateStreetStatsForTrack"));

  if (line.empty())
    return;

  m2::PointD prev = geometry::GetPoint(line[0]);
  for (size_t i = 1; i < line.size(); ++i)
  {
    auto const & ptWithAlt = line[i];
    m2::PointD curr = geometry::GetPoint(ptWithAlt);
    double distMerc = (curr - prev).Length();
    double distMeters = mercator::DistanceOnEarth(prev, curr);
    size_t segments = std::max<size_t>(1, static_cast<size_t>(std::ceil(distMeters / 10.0)));  // Sample every 10m
    m2::PointD dir = (curr - prev).Normalize();
    double step = distMerc / segments;
    for (size_t s = 0; s <= segments; ++s)
    {
      m2::PointD p = prev + dir * (s * step);
      auto const latlon = mercator::ToLatLon(p);
      UpdateStreetStats(latlon.m_lat, latlon.m_lon, 1);
    }
    prev = curr;
  }
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
  double const lat_rad = math::DegToRad(lat);
  double const lon_rad = math::DegToRad(lon);
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
  LOG(LINFO, ("OnLocationUpdate"));

  std::set<std::int64_t> pixels;
  AddPixelsInRadius(info.m_latitude, info.m_longitude, pixels);
  size_t numNewlyExploredPixels = 0;
  for (auto const & pix : pixels)
  {
    auto * pixel = FindStreetPixel(pix);
    if (pixel == nullptr || pixel->IsExplored())
      continue;
    pixel->SetExplored(true);
    msync(pixel, sizeof(df::StreetPixel), MS_ASYNC);
    numNewlyExploredPixels++;

    if (!m_accountedBits.empty())
    {
      size_t idx = GetPixelIndex(pixel);
      if (!IsAccountedIndex(idx))
      {
        SetAccountedIndex(idx);
        if (m_explorationListener)
        {
          ExplorationDelta d;
          {
            std::lock_guard<std::mutex> lock(m_countryIdMutex);
            d.m_regionId = m_countryId;
          }
          d.m_newPixels = 1;
          d.m_eventTimeSec = info.m_timestamp;
          m_explorationListener(d);
        }
      }
    }
  }

  UpdateStreetStats(info.m_latitude, info.m_longitude, numNewlyExploredPixels);

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
    size_t const maxPixels = 10;  // Limit to avoid too long vibration
    size_t const count = std::min(numNewlyExploredPixels, maxPixels);

    std::vector<uint32_t> durations(count, 30);
    std::vector<uint32_t> delays(count, 20);

    platform::VibratePattern(durations.data(), delays.data(), count);
  }
}

void StreetPixelsManager::UpdateStreetStats(double lat, double lon, size_t numNewlyExploredPixels)
{
  // LOG(LINFO, ("UpdateStreetStats", lat, lon, numNewlyExploredPixels));

  if (numNewlyExploredPixels == 0)
    return;

  MwmSet::MwmId mwmId;
  {
    std::lock_guard<std::mutex> lock(m_countryIdMutex);
    if (m_countryId.empty())
      return;
    mwmId = m_dataSource.GetMwmIdByCountryFile(platform::CountryFile(m_countryId));
  }

  if (!mwmId.IsAlive())
    return;

  m2::PointD centerMercator = mercator::FromLatLon(lat, lon);

  std::map<FeatureID, std::set<uint32_t>> featureUpdates;

  indexer::ForEachFeatureAtPoint(
    m_dataSource,
    [&](FeatureType & ft)
    {
      if (!IsExplorable(ft))
        return;

      ft.ParseGeometry(FeatureType::BEST_GEOMETRY);
      size_t const pointsCount = ft.GetPointsCount();
      if (pointsCount < 2)
        return;

      double minSqDist = std::numeric_limits<double>::max();
      double distanceAlongFeatureM = -1.0;
      double accumulatedLengthM = 0.0;

      for (size_t i = 1; i < pointsCount; ++i)
      {
        m2::PointD const p1 = ft.GetPoint(i - 1);
        m2::PointD const p2 = ft.GetPoint(i);

        m2::ParametrizedSegment<m2::PointD> segment(p1, p2);
        m2::PointD const closestPoint = segment.ClosestPointTo(centerMercator);
        double const sqDist = centerMercator.SquaredLength(closestPoint);

        if (sqDist < minSqDist)
        {
          minSqDist = sqDist;
          distanceAlongFeatureM = accumulatedLengthM + mercator::DistanceOnEarth(p1, closestPoint);
        }
        accumulatedLengthM += mercator::DistanceOnEarth(p1, p2);
      }

      if (distanceAlongFeatureM >= 0)
      {
        uint32_t const pixelIndex = static_cast<uint32_t>(distanceAlongFeatureM / kSegmentLengthMeters);
        featureUpdates[ft.GetID()].insert(pixelIndex);
      }
    },
    centerMercator);

  if (featureUpdates.empty())
    return;

  auto & db = street_stats::StreetStatsDB::Instance();
  for (auto const & [fid, pixelIndices] : featureUpdates)
  {
    auto bitmask = db.GetBitmask(fid.m_mwmId, fid.m_index);
    // If bitmask does not exist, it means the stats for this MWM have not been generated.
    // We should not create it on the fly, as we don't know the full feature length.
    if (!bitmask)
      continue;

    bool updated = false;
    for (uint32_t pixelIndex : pixelIndices)
    {
      size_t const byteIndex = pixelIndex / 8;
      if (byteIndex < bitmask->size())
      {
        uint8_t const bitIndex = pixelIndex % 8;
        if (!((*bitmask)[byteIndex] & (1 << bitIndex)))
        {
          (*bitmask)[byteIndex] |= (1 << bitIndex);
          updated = true;
        }
      }
    }

    if (updated)
      db.SaveBitmask(fid.m_mwmId, fid.m_index, *bitmask);
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
  LOG(LINFO, ("OnUpdateCurrentCountry", countryId));

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
                          LOG(LINFO, ("Loading street pixels in background thread because country changed to", countryId));
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
  LOG(LINFO, ("LoadExploredFractions"));

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
  LOG(LINFO, ("SaveExploredFractions"));

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
  m_accountedBits.clear();
  m_accountedDirty = false;

  {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    ChangeState(StreetPixelsState{m_state.enabled, StreetPixelsStatus::NotReady});
  }
}

std::string StreetPixelsManager::GetAccountedFilePath() const
{
  storage::CountryId country;
  {
    std::lock_guard<std::mutex> lock(m_countryIdMutex);
    country = m_countryId;
  }
  return GetPlatform().WritablePathForFile(country + ".pixa");
}

size_t StreetPixelsManager::GetPixelIndex(df::StreetPixel const * ptr) const
{
  if (m_streetPixels.empty())
    return 0;
  return ptr - m_streetPixels.data();
}

bool StreetPixelsManager::IsAccountedIndex(size_t idx) const
{
  if (m_accountedBits.empty() || idx >= m_accountedBits.size() * 8)
    return false;
  size_t byteIdx = idx / 8;
  size_t bitIdx = idx % 8;
  return (m_accountedBits[byteIdx] & (1 << bitIdx)) != 0;
}

void StreetPixelsManager::SetAccountedIndex(size_t idx)
{
  if (idx >= m_streetPixels.size())
    return;

  // Resize m_accountedBits if necessary
  size_t requiredBytes = (idx + 8) / 8;  // +8 to round up
  if (m_accountedBits.size() < requiredBytes)
    m_accountedBits.resize(requiredBytes, 0);

  size_t byteIdx = idx / 8;
  size_t bitIdx = idx % 8;
  m_accountedBits[byteIdx] |= (1 << bitIdx);
  m_accountedDirty = true;
}

void StreetPixelsManager::LoadAccountedBits()
{
  LOG(LINFO, ("LoadAccountedBits"));

  std::string path = GetAccountedFilePath();
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open())
  {
    LOG(LINFO, ("No accounted bits file for", GetCurrentCountryId()));
    return;
  }

  ifs.seekg(0, std::ios::end);
  size_t fileSize = ifs.tellg();
  ifs.seekg(0, std::ios::beg);

  m_accountedBits.resize(fileSize);
  ifs.read(reinterpret_cast<char *>(m_accountedBits.data()), fileSize);
  m_accountedDirty = false;

  LOG(LINFO, ("Loaded", fileSize, "bytes of accounted bits for", GetCurrentCountryId()));
}

void StreetPixelsManager::SaveAccountedBits()
{
  LOG(LINFO, ("SaveAccountedBits"));
  if (!m_accountedDirty)
    return;

  std::string path = GetAccountedFilePath();
  std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
  if (!ofs.is_open())
  {
    LOG(LWARNING, ("Failed to open accounted bits file for writing:", path));
    return;
  }

  ofs.write(reinterpret_cast<char const *>(m_accountedBits.data()), m_accountedBits.size());
  m_accountedDirty = false;

  LOG(LINFO, ("Saved", m_accountedBits.size(), "bytes of accounted bits for", GetCurrentCountryId()));
}