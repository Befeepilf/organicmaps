#pragma once

#include "drape_frontend/circles_pack_shape.hpp"
#include "drape_frontend/frame_values.hpp"
#include "drape_frontend/street_pixel_point.hpp"

#include "shaders/program_manager.hpp"

#include "drape/graphics_context.hpp"
#include "drape/pointers.hpp"

#include "geometry/screenbase.hpp"

#include <functional>
#include <map>
#include <vector>

namespace df
{
class StreetPixelRenderer final
{
public:
  using TRenderDataRequestFn = std::function<void(uint32_t)>;
  explicit StreetPixelRenderer(TRenderDataRequestFn const & dataRequestFn);

  void AddRenderData(ref_ptr<dp::GraphicsContext> context, ref_ptr<gpu::ProgramManager> mng,
                     drape_ptr<CirclesPackRenderData> && renderData);

  void Render(ref_ptr<dp::GraphicsContext> context, ref_ptr<gpu::ProgramManager> mng, ScreenBase const & screen,
              int zoomLevel, FrameValues const & frameValues);

  void UpdatePixels(std::vector<StreetPixelPoint> const & toAdd, std::vector<int64_t> const & toRemove);

  void Update();
  void Clear();
  void ClearRenderData();

private:
  TRenderDataRequestFn m_dataRequestFn;
  std::vector<drape_ptr<CirclesPackRenderData>> m_renderData;
  std::map<int64_t, StreetPixelPoint> m_points;

  bool m_needUpdate;
  bool m_waitForRenderData;
  std::vector<std::pair<CirclesPackHandle *, size_t>> m_handlesCache;
  float m_radius;
  m2::PointD m_pivot;
};
}  // namespace df
