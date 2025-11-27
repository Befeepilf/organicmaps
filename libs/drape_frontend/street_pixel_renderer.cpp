#include "drape_frontend/street_pixel_renderer.hpp"
#include "drape_frontend/map_shape.hpp"
#include "drape_frontend/shape_view_params.hpp"
#include "drape_frontend/tile_utils.hpp"
#include "drape_frontend/visual_params.hpp"

#include "shaders/programs.hpp"

#include "drape/utils/vertex_decl.hpp"
#include "drape/vertex_array_buffer.hpp"

#include <algorithm>
#include <span>
#include <unordered_map>
#include <vector>

namespace df
{
namespace
{
uint32_t const kAveragePointsCount = 2048;
int const kMinVisibleZoomLevel = 9;

// Zoom level used for grouping points into buckets.
// Must be >= kMinVisibleZoomLevel so that each visible viewport covers limited number of buckets.
int const kBucketZoomLevel = 15;

// Radius of circles depending on zoom levels.
std::array<float, 20> const kRadiusInPixel = {
  // 1   2     3     4     5     6     7     8     9     10
  0.6f, 0.8f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
  // 11   12    13    14    15    16    17    18    19     20
  1.0f, 1.0f, 1.0f, 1.0f, 1.5f, 2.0f, 2.0f, 3.0f, 4.0f, 5.5f};
}  // namespace

StreetPixelRenderer::StreetPixelRenderer(TRenderDataRequestFn const & dataRequestFn)
  : m_dataRequestFn(dataRequestFn)
  , m_needUpdate(false)
  , m_waitForRenderData(false)
  , m_radius(0.0f)
  , m_enabled(false)
{}

void StreetPixelRenderer::SetEnabled(bool enabled)
{
  m_enabled = enabled;
  if (!m_enabled)
    ClearRenderData();
}

void StreetPixelRenderer::AddRenderData(ref_ptr<dp::GraphicsContext> context, ref_ptr<gpu::ProgramManager> mng,
                                        drape_ptr<CirclesPackRenderData> && renderData)
{
  drape_ptr<CirclesPackRenderData> data = std::move(renderData);
  ref_ptr<dp::GpuProgram> program = mng->GetProgram(gpu::Program::CirclePoint);
  data->m_bucket->GetBuffer()->Build(context, program);
  m_renderData.push_back(std::move(data));
  m_waitForRenderData = false;
}

void StreetPixelRenderer::ClearRenderData()
{
  m_renderData.clear();
  m_handlesCache.clear();
  m_waitForRenderData = false;
  m_needUpdate = true;
}

void StreetPixelRenderer::UpdatePixels(std::span<StreetPixel> const & toAdd)
{
  m_allPixels = toAdd;
  bool wasChanged = false;

  if (!toAdd.empty())
  {
    LOG(LINFO, ("Adding", toAdd.size()));
    for (std::uint32_t i = 0; i < toAdd.size(); i++)
    {
      StreetPixel const & pixel = toAdd[i];
      TileKey const bucketKey = GetTileKeyByPoint(pixel.GetPoint(), kBucketZoomLevel);
      m_tileBuckets[bucketKey].push_back(i);
    }
    wasChanged = true;
  }
  else
  {
    LOG(LINFO, ("No additions"));
  }

  m_needUpdate = wasChanged;
}

void StreetPixelRenderer::Render(ref_ptr<dp::GraphicsContext> context, ref_ptr<gpu::ProgramManager> mng,
                                 ScreenBase const & screen, int zoomLevel, FrameValues const & frameValues)
{
  if (!m_enabled)
    return;

  if (zoomLevel < kMinVisibleZoomLevel)
  {
    ClearRenderData();
    return;
  }

  if (m_tileBuckets.empty())
    return;

  if (m_needUpdate)
  {
    LOG(LINFO, ("Reconstruct render data"));

    // Check if there are render data.
    if (m_renderData.empty() && !m_waitForRenderData)
    {
      m_dataRequestFn(kAveragePointsCount);
      m_waitForRenderData = true;
    }

    if (m_waitForRenderData)
      return;

    ASSERT(!m_renderData.empty(), ());
    m_handlesCache.clear();
    for (size_t i = 0; i < m_renderData.size(); i++)
    {
      auto & bucket = m_renderData[i]->m_bucket;
      ASSERT_EQUAL(bucket->GetOverlayHandlesCount(), 1, ());
      CirclesPackHandle * handle = static_cast<CirclesPackHandle *>(bucket->GetOverlayHandle(0).get());
      handle->Clear();
      m_handlesCache.push_back(std::make_pair(handle, 0));
    }

    m_radius = CalculateRadius(screen, kRadiusInPixel);

    m_pivot = screen.GlobalRect().Center();
    size_t cacheIndex = 0;

    std::unordered_set<std::uint64_t> renderedExploredPixels;
    std::unordered_set<std::uint64_t> renderedUnexploredPixels;

    // Determine tiles that intersect the screen clip rect in bucket zoom level.
    CalcTilesCoverage(screen.ClipRect(), kBucketZoomLevel,
                      [this, &screen, &zoomLevel, &cacheIndex, &renderedExploredPixels, &renderedUnexploredPixels](int tileX, int tileY)
                      {
                        if (m_waitForRenderData)
                          return;

                        TileKey const key(tileX, tileY, kBucketZoomLevel);
                        auto itBucket = m_tileBuckets.find(key);
                        if (itBucket == m_tileBuckets.end())
                          return;

                        m2::RectD const clipRect = screen.ClipRect();
                        for (std::uint32_t i : itBucket->second)
                        {
                          StreetPixel const & pixel = m_allPixels[i];

                          m2::PointD const & pixelPoint = pixel.GetPoint();
                          if (!clipRect.IsPointInside(pixelPoint))
                            continue;

                          // Street pixels are very dense. Reduce density at lower zoom levels to improve rendering performance.
                          // Takes advantage of the nested scheme where 2bits are added to the healpix id for each nside increment.
                          if (zoomLevel < 14) {
                            int const shift = 2 * (15 - zoomLevel);
                            if (shift > 0) {
                              std::uint64_t const parentPixelId = pixel.GetPixelId() >> shift;

                              if (renderedExploredPixels.contains(parentPixelId) || renderedUnexploredPixels.contains(parentPixelId))
                                continue;
                              if (pixel.IsExplored())
                                renderedExploredPixels.insert(parentPixelId);
                              else
                                renderedUnexploredPixels.insert(parentPixelId);
                            }
                          }

                          m2::PointD const convertedPoint =
                            MapShape::ConvertToLocal(pixelPoint, m_pivot, kShapeCoordScalar);

                          auto & cacheHandle = m_handlesCache[cacheIndex];
                          cacheHandle.first->SetPoint(cacheHandle.second, convertedPoint, m_radius, pixel.GetColor());
                          cacheHandle.second++;
                          if (cacheHandle.second >= cacheHandle.first->GetPointsCount())
                            cacheIndex++;

                          if (cacheIndex >= m_handlesCache.size())
                          {
                            m_dataRequestFn(kAveragePointsCount);
                            m_waitForRenderData = true;
                            return;
                          }
                        }
                      });

    if (m_waitForRenderData)
      return;

    m_needUpdate = false;
  }

  if (m_handlesCache.empty() || m_handlesCache.front().second == 0)
    return;

  ASSERT_LESS_OR_EQUAL(m_renderData.size(), m_handlesCache.size(), ());

  // Render points.
  gpu::MapProgramParams params;
  frameValues.SetTo(params);
  math::Matrix<float, 4, 4> mv = screen.GetModelView(m_pivot, kShapeCoordScalar);
  params.m_modelView = glsl::make_mat4(mv.m_data);
  ref_ptr<dp::GpuProgram> program = mng->GetProgram(gpu::Program::CirclePoint);
  program->Bind();

  ASSERT_GREATER(m_renderData.size(), 0, ());
  dp::RenderState const & state = m_renderData.front()->m_state;
  dp::ApplyState(context, program, state);
  mng->GetParamsSetter()->Apply(context, program, params);

  for (size_t i = 0; i < m_renderData.size(); i++)
  {
    if (m_handlesCache[i].second != 0)
      m_renderData[i]->m_bucket->Render(context, state.GetDrawAsLine());
  }
}

void StreetPixelRenderer::Update() { m_needUpdate = true; }

void StreetPixelRenderer::Clear()
{
  m_allPixels = {};
  m_tileBuckets.clear();
  ClearRenderData();
}
}  // namespace df