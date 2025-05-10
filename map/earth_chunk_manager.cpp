#include "map/earth_chunk_manager.hpp"

#include "base/macros.hpp"
#include "geometry/point2d.hpp"
#include "geometry/polyline2d.hpp"

#include <healpix_base.h>

EarthChunkManager::EarthChunkManager(df::DrapeApi & drapeApi)
  : m_drapeApi(drapeApi)
  , m_hpNest()
  , m_polylines(0)
{
  m_hpNest.SetNside(1024, Healpix_Ordering_Scheme::NEST);
  m_nPixels = m_hpNest.Npix();
}

void EarthChunkManager::LoadEarthChunks()
{
  CHECK_THREAD_CHECKER(m_threadChecker, ());
  CHECK(!m_loadEarthChunksCalled, ("LoadEarthChunks should be called only once."));
  //   m_polylines.resize(m_nPixels);
  for (int64_t i = 0; i < m_nPixels; ++i)
  {
    std::vector<vec3> out(4);
    m_hpNest.boundaries(i, 1, out);
    std::vector<m2::PointD> poly(4);
    for (size_t j = 0; j < 4; ++j)
    {
      points[j] = m2::PointD(out[j].x, out[j].y);
    }

    static dp::Color const orangeColor = dp::Color(242, 138, 2, 255);
    m_drapeApi.AddLine(id, df::DrapeApiLineData(poly, orangeColor).Width(7.0f).ShowPoints(true).ShowId());
  }
  m_loadEarthChunksCalled = true;
}
