#include "map/street_pixel_manager.hpp"

#include "drape_frontend/drape_engine.hpp"
#include "drape_frontend/selection_shape.hpp"

#include "indexer/classificator.hpp"
#include "indexer/feature_decl.hpp"
#include "indexer/features_vector.hpp"

#include "geometry/angles.hpp"
#include "geometry/mercator.hpp"
#include "geometry/point2d.hpp"

#include "platform/platform.hpp"

#include "base/assert.hpp"
#include "base/logging.hpp"
#include "base/math.hpp"
#include "base/src_point.hpp"

#include <healpix_base.h>
#include <sstream>
#include <string>

StreetPixelManager::StreetPixelManager()
{
  m_layer = std::make_unique<UserMarkLayer>(UserMark::STREET_PIXEL);
  m_visibleGroups.insert(static_cast<kml::MarkGroupId>(UserMark::STREET_PIXEL));
  m_healpixBase.SetNside(1048576, Healpix_Ordering_Scheme::RING);
}

void StreetPixelManager::SetDrapeEngine(ref_ptr<df::DrapeEngine> engine)
{
  m_drapeEngine.Set(engine);
  if (engine)
    NotifyDrape();
}

void StreetPixelManager::SetBookmarkManager(BookmarkManager * bmManager) { m_bmManager = bmManager; }

void StreetPixelManager::LoadStreetPixelsForRegion(storage::CountryId const & countryId,
                                                   storage::LocalFilePtr const & localFile)
{
  if (countryId == "World" || countryId == "WorldCoasts")
  {
    LOG(LINFO, ("Skipping country file for", countryId));
    return;
  }

  std::vector<int64> streetPixels;

  std::string filePath = GetPlatform().WritablePathForFile(countryId + ".pix");

  try
  {
    LOG(LINFO, ("Trying to load existing pix file for", countryId));
    base::FileData file(filePath, base::FileData::Op::READ);
    uint64_t const fileSizeBytes = file.Size();
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
    writer->Write(streetPixels.data(), streetPixels.size() * sizeof(int64));
    writer->Flush();
    writer.reset();
  }

  LOG(LINFO, ("Converting", streetPixels.size(), "pixels to mercator points"));
  std::vector<::m2::PointD> streetPixelPoints(streetPixels.size());
  for (size_t i = 0; i < streetPixels.size(); ++i)
  {
    auto const pixel = streetPixels[i];
    pointing const ptg = m_healpixBase.pix2ang(pixel);
    double const lat = 90.0 - base::RadToDeg(ptg.theta);
    double const lon = base::RadToDeg(ptg.phi);
    streetPixelPoints[i] = mercator::FromLatLon(lat, lon);
  }
  LOG(LINFO, ("Adding", streetPixelPoints.size(), "pixels"));
  AddPixels(streetPixelPoints);
  LOG(LINFO, ("Done."));
}

void StreetPixelManager::DeriveStreetPixelsFromFeatures(FeaturesVectorTest & featuresVector,
                                                        std::vector<int64> & streetPixels)
{
  std::vector<m2::PointD> points;
  Classificator & c = classif();

  int numStreets = 0;
  featuresVector.GetVector().ForEach(
    [&](FeatureType & feature, uint32_t)
    {
      if (feature.GetGeomType() != feature::GeomType::Line)
        return;

      bool isExplorable = false;
      feature.ForEachType(
        [&](uint32_t type)
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

      ::m2::PointD prevPoint = feature.GetPoint(0);
      for (size_t i = 1; i < numPoints; ++i)
      {
        auto const point = feature.GetPoint(i);
        points.push_back(prevPoint);

        if (::m2::AlmostEqualAbs(prevPoint, point, 1e-6))
        {
          continue;
        }

        ::m2::PointD const p12 = point - prevPoint;
        ::m2::PointD const p12Norm = p12.Normalize();

        double const distanceMercator = p12.Length();
        double const distanceMeters = mercator::DistanceOnEarth(prevPoint, point);
        // segmentize into 10 meter segments
        size_t const numSegments = std::ceil(distanceMeters / 10.0);
        double const segmentSize = distanceMercator / numSegments;
        for (size_t segment = 1; segment < numSegments; segment++)
        {
          ::m2::PointD const segmentPoint = prevPoint + p12Norm * (segment * segmentSize);
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
    pointing ptg(acos(sin(lat_rad)), lon_rad);
    streetPixels.emplace_back(m_healpixBase.ang2pix(ptg));
  }

  std::vector<int64>::iterator ip = std::unique(streetPixels.begin(), streetPixels.end());
  streetPixels.resize(std::distance(streetPixels.begin(), ip));  // remove duplicates
  LOG(LINFO, ("Found", streetPixels.size(), "street pixels for", numStreets, "streets"));
}

void StreetPixelManager::AddPixels(std::vector<::m2::PointD> const & points)
{
  for (auto const & point : points)
  {
    auto mark = std::make_unique<PixelMark>(point);
    auto * ptr = mark.get();
    kml::MarkId const id = ptr->GetId();

    m_allMarks.insert(id);
    m_createdMarks.insert(id);
    m_marks.emplace(id, std::move(mark));
    m_layer->AttachUserMark(id);
  }
  m_updatedGroups.insert(static_cast<kml::MarkGroupId>(UserMark::STREET_PIXEL));
  NotifyDrape();
}

df::UserPointMark const * StreetPixelManager::GetUserPointMark(kml::MarkId id) const
{
  auto it = m_marks.find(id);
  return it == m_marks.end() ? nullptr : it->second.get();
}

void StreetPixelManager::NotifyDrape()
{
  if (!m_drapeEngine)
    return;

  m_drapeEngine.SafeCall(&df::DrapeEngine::UpdateUserMarks, this, m_firstNotify);
  m_firstNotify = false;

  m_createdMarks.clear();
  m_removedMarks.clear();
  m_updatedMarks.clear();
  m_updatedGroups.clear();
}
