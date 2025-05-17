#include "map/earth_chunk_manager.hpp"
#include "map/framework.hpp"

#include "geometry/angles.hpp"
#include "geometry/mercator.hpp"

#include "drape_frontend/drape_api.hpp"

#include "base/logging.hpp"
#include "base/thread_checker.hpp"

#include <healpix_base.h>

#include "base/math.hpp"
#include "geometry/latlon.hpp"
#include "geometry/region2d.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

EarthChunkManager::EarthChunkManager()
  : m_healpixBase()
{
  m_healpixBase.SetNside(8, Healpix_Ordering_Scheme::RING);
}

void EarthChunkManager::SetDrapeApi(ref_ptr<df::DrapeApi> drapeApi)
{
  CHECK_THREAD_CHECKER(m_threadChecker, ());
  m_drapeApi = drapeApi;
}

void EarthChunkManager::LoadEarthChunks()
{
  CHECK_THREAD_CHECKER(m_threadChecker, ());
  CHECK(!m_loadEarthChunksCalled, ("LoadEarthChunks should be called only once."));
  m_loadEarthChunksCalled = true;

  // HEALPix base parameters.
  // Coordinates returned by pix2xyf are expressed in the \[0, nside) range for every face.
  // Using the total number of pixels (npix) instead of nside dramatically shrinks the
  // coordinate values and produces a cluster of almost coincident cells around the
  // zero meridian. Here we convert them with the correct divisor.
  int64 const nside = m_healpixBase.Nside();
  double const nsideD = static_cast<double>(nside);
  double const dc = 0.5 / nsideD;  // Half-cell offset in the face reference frame.

  int64 const npix = m_healpixBase.Npix();

  std::vector<m2::PointD> points(4);
  int ix, iy, face;
  double z, phi, sth;
  bool have_sth;
  for (int64 ipix = 0; ipix < npix; ++ipix)
  {
    LOG(LINFO, ("Processing chunk", ipix, "of", npix));

    points.clear();
    points.resize(4);

    m_healpixBase.pix2xyf(ipix, ix, iy, face);
    double const xc = (ix + 0.5) / nsideD;
    double const yc = (iy + 0.5) / nsideD;

    // Bottom-left
    m_healpixBase.xyf2loc(xc - dc, yc - dc, face, z, phi, sth, have_sth);
    points[0] = mercator::FromLatLon(90.0 - base::RadToDeg(acos(z)), base::RadToDeg(phi) - 180.0);

    // Bottom-right
    m_healpixBase.xyf2loc(xc + dc, yc - dc, face, z, phi, sth, have_sth);
    points[1] = mercator::FromLatLon(90.0 - base::RadToDeg(acos(z)), base::RadToDeg(phi) - 180.0);

    // Top-right
    m_healpixBase.xyf2loc(xc + dc, yc + dc, face, z, phi, sth, have_sth);
    points[2] = mercator::FromLatLon(90.0 - base::RadToDeg(acos(z)), base::RadToDeg(phi) - 180.0);

    // Top-left
    m_healpixBase.xyf2loc(xc - dc, yc + dc, face, z, phi, sth, have_sth);
    points[3] = mercator::FromLatLon(90.0 - base::RadToDeg(acos(z)), base::RadToDeg(phi) - 180.0);

    // Close the polygon by repeating the first point.
    points.push_back(points.front());

    dp::Color const color{255, 0, 0, 255};
    std::string const baseId = "chunk_" + std::to_string(ipix);

    auto const addLine = [this, &color](std::string const & lineId, std::vector<m2::PointD> const & pts)
    {
      if (pts.size() < 2 || !m_drapeApi)
        return;

      m_drapeApi->AddLine(lineId, df::DrapeApiLineData(pts, color).Width(3.0f).ShowPoints(false));
    };

    double minLon = points[0].x, maxLon = points[0].x;
    for (auto const & p : points)
    {
      minLon = std::min(minLon, p.x);
      maxLon = std::max(maxLon, p.x);
    }

    // Dateline crossing if the span is bigger than half-globe.
    bool const crossesDateline = (maxLon - minLon) > 180.0;

    if (!crossesDateline)
    {
      LOG(LINFO, ("Adding line with id:", baseId));
      addLine(baseId, points);
    }
    else
    {
      // Split into two polylines by inserting points on the dateline.
      std::vector<m2::PointD> east, west;
      auto pushDateline = [&](double lat, bool toEast)
      {
        m2::PointD eastPt(180.0, lat);
        m2::PointD westPt(-180.0, lat);
        if (toEast)
        {
          east.push_back(eastPt);
          west.push_back(westPt);
        }
        else
        {
          west.push_back(westPt);
          east.push_back(eastPt);
        }
      };

      auto belongsEast = [](double lon) { return lon >= 0.0; };

      for (size_t i = 0; i + 1 < points.size(); ++i)
      {
        auto const & p1 = points[i];
        auto const & p2 = points[i + 1];

        bool const p1East = belongsEast(p1.x);
        bool const p2East = belongsEast(p2.x);

        if (p1East)
          east.push_back(p1);
        else
          west.push_back(p1);

        // Check if the segment crosses the dateline.
        if (std::fabs(p1.x - p2.x) > 180.0)
        {
          // Segment is almost horizontal so use p1.y as lat for intersection.
          double const lat = p1.y;
          pushDateline(lat, p1East);
        }
      }

      // Add the last point (duplicate of first) to the appropriate collection.
      if (belongsEast(points.back().x))
        east.push_back(points.back());
      else
        west.push_back(points.back());

      addLine(baseId + "_E", east);
      addLine(baseId + "_W", west);
    }
  }
}
