#pragma once

#include "base/thread_checker.hpp"
#include "drape_frontend/drape_api.hpp"

#include <healpix_base.h>

class EarthChunkManager final
{
public:
  explicit EarthChunkManager();

  void SetDrapeApi(ref_ptr<df::DrapeApi> drapeApi);
  void LoadEarthChunks();

private:
  bool m_loadEarthChunksCalled = false;
  ThreadChecker m_threadChecker;
  ref_ptr<df::DrapeApi> m_drapeApi = nullptr;
  T_Healpix_Base<int64> m_healpixBase;
};
