#pragma once

#include "base/thread_checker.hpp"
#include "drape_frontend/drape_api.hpp"

#include <healpix_base.h>

class EarthChunkManager final
{
public:
  explicit EarthChunkManager(df::DrapeApi & drapeApi);

  void LoadEarthChunks();

private:
  df::DrapeApi & m_drapeApi;
  bool m_loadEarthChunksCalled = false;
  ThreadChecker m_threadChecker;
  T_Healpix_Base<int64_t> m_hpNest;
  int64_t m_nPixels;
  //   std::vector<sPolyline2d> m_polylines;
};
