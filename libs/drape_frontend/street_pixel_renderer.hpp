#pragma once

#include "drape_frontend/circles_pack_shape.hpp"
#include "drape_frontend/frame_values.hpp"
#include "drape_frontend/street_pixel.hpp"

#include "shaders/program_manager.hpp"

#include "drape/graphics_context.hpp"
#include "drape/pointers.hpp"

#include "geometry/screenbase.hpp"

#include "drape_frontend/tile_utils.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <span>
#include <unordered_map>
#include <vector>

namespace df
{
struct TileKeyHasher
{
  size_t operator()(TileKey const & key) const noexcept
  {
    // Simple combination of coordinates and zoom level.
    // We do not include generations because buckets are built on a fixed zoom level.
    size_t seed = std::hash<int>()(key.m_x);
    seed ^= std::hash<int>()(key.m_y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::uint8_t>()(key.m_zoomLevel) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

class StreetPixelRenderer final
{
public:
  void SetEnabled(bool enabled);

  using TRenderDataRequestFn = std::function<void(uint32_t)>;
  explicit StreetPixelRenderer(TRenderDataRequestFn const & dataRequestFn);

  void AddRenderData(ref_ptr<dp::GraphicsContext> context, ref_ptr<gpu::ProgramManager> mng,
                     drape_ptr<CirclesPackRenderData> && renderData);

  void Render(ref_ptr<dp::GraphicsContext> context, ref_ptr<gpu::ProgramManager> mng, ScreenBase const & screen,
              int zoomLevel, FrameValues const & frameValues);

  void UpdatePixels(std::span<StreetPixel> const & toAdd);

  void Update();
  void Clear();
  void ClearRenderData();

private:
  TRenderDataRequestFn m_dataRequestFn;
  std::vector<drape_ptr<CirclesPackRenderData>> m_renderData;

  std::unordered_map<TileKey, std::vector<std::uint32_t>, TileKeyHasher> m_tileBuckets;
  std::span<StreetPixel> m_allPixels;
  bool m_needUpdate;
  bool m_waitForRenderData;
  std::vector<std::pair<CirclesPackHandle *, size_t>> m_handlesCache;
  float m_radius;
  m2::PointD m_pivot;
  bool m_enabled;
};
}  // namespace df