#include "drape_frontend/street_pixel_renderer.hpp"
#include "drape_frontend/map_shape.hpp"
#include "drape_frontend/shape_view_params.hpp"
#include "drape_frontend/tile_utils.hpp"
#include "drape_frontend/visual_params.hpp"

#include "shaders/programs.hpp"

#include "drape/utils/vertex_decl.hpp"
#include "drape/vertex_array_buffer.hpp"

#include <algorithm>
#include <unordered_map>
#include <vector>

namespace df
{
namespace
{
int const kMinVisibleZoomLevel = 10;
uint32_t const kAveragePointsCount = 2048;

// Zoom level used for grouping points into buckets.
// Must be >= kMinVisibleZoomLevel so that each visible viewport covers limited number of buckets.
int const kBucketZoomLevel = 15;
}  // namespace

StreetPixelRenderer::StreetPixelRenderer(TRenderDataRequestFn const & dataRequestFn)
  : m_dataRequestFn(dataRequestFn)
  , m_needUpdate(true)
  , m_waitForRenderData(false)
  , m_radius(4.0f)
{}

void StreetPixelRenderer::AddRenderData(ref_ptr<dp::GraphicsContext> context, ref_ptr<gpu::ProgramManager> mng,
                                        drape_ptr<CirclesPackRenderData> && renderData)
{
  drape_ptr<CirclesPackRenderData> data = std::move(renderData);
  ref_ptr<dp::GpuProgram> program = mng->GetProgram(gpu::Program::CirclePoint);
  data->m_bucket->GetBuffer()->Build(context, program);
  m_renderData.push_back(std::move(data));
  m_waitForRenderData = false;
  m_needUpdate = true;
}

void StreetPixelRenderer::ClearRenderData()
{
  m_renderData.clear();
  m_handlesCache.clear();
  m_waitForRenderData = false;
  m_needUpdate = true;
}

void StreetPixelRenderer::UpdatePixels(std::vector<StreetPixelPoint> const & toAdd,
                                       std::vector<StreetPixelPoint> const & toRemove)
{
  bool wasChanged = false;

  // Handle removals first.
  if (!toRemove.empty())
  {
    for (df::StreetPixelPoint point : toRemove)
    {
      // Remove from bucket container as well.
      TileKey const bucketKey = GetTileKeyByPoint(point, kBucketZoomLevel);
      auto itBucket = m_tileBuckets.find(bucketKey);
      if (itBucket != m_tileBuckets.end())
      {
        auto & vec = itBucket->second;
        auto const pointId = point.GetPixelId();
        vec.erase(std::remove_if(vec.begin(), vec.end(),
                                 [pointId](StreetPixelPoint const & p) { return p.GetPixelId() == pointId; }),
                  vec.end());
        if (vec.empty())
          m_tileBuckets.erase(itBucket);
      }
    }
    wasChanged = true;
  }

  // Handle additions.
  if (!toAdd.empty())
  {
    for (auto const & pt : toAdd)
    {
      TileKey const bucketKey = GetTileKeyByPoint(pt, kBucketZoomLevel);
      m_tileBuckets[bucketKey].push_back(pt);
    }
    wasChanged = true;
  }

  if (wasChanged)
    m_needUpdate = true;
}

void StreetPixelRenderer::Update() { m_needUpdate = true; }

void StreetPixelRenderer::Render(ref_ptr<dp::GraphicsContext> context, ref_ptr<gpu::ProgramManager> mng,
                                 ScreenBase const & screen, int zoomLevel, FrameValues const & frameValues)
{
  if (zoomLevel < kMinVisibleZoomLevel)
  {
    Clear();
    return;
  }

  if (m_tileBuckets.empty())
    return;

  if (m_needUpdate)
  {
    // Check if there are render data.
    if (m_renderData.empty() && !m_waitForRenderData)
    {
      m_dataRequestFn(kAveragePointsCount);
      m_waitForRenderData = true;
    }

    if (m_waitForRenderData)
      return;

    m_pivot = screen.GlobalRect().Center();

    // Update points' positions and colors.
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

    double const currentScaleGtoP = 1.0 / screen.GetScale();
    double const radiusMercator = m_radius / currentScaleGtoP;

    size_t cacheIndex = 0;

    // Determine tiles that intersect the screen clip rect in bucket zoom level.
    CalcTilesCoverage(screen.ClipRect(), kBucketZoomLevel,
                      [this, &screen, radiusMercator, &cacheIndex](int tileX, int tileY)
                      {
                        TileKey const key(tileX, tileY, kBucketZoomLevel);
                        auto itBucket = m_tileBuckets.find(key);
                        if (itBucket == m_tileBuckets.end())
                          return;

                        // If we have already requested additional render data during this update pass, skip
                        // processing of the remaining tiles. The new data will arrive on the next frame.
                        if (m_waitForRenderData)
                          return;

                        for (StreetPixelPoint const & pt : itBucket->second)
                        {
                          m2::RectD pointRect(pt.x - radiusMercator, pt.y - radiusMercator, pt.x + radiusMercator,
                                              pt.y + radiusMercator);
                          if (!screen.ClipRect().IsIntersect(pointRect))
                            continue;

                          m2::PointD const convertedPt = MapShape::ConvertToLocal(pt, m_pivot, kShapeCoordScalar);

                          m_handlesCache[cacheIndex].first->SetPoint(m_handlesCache[cacheIndex].second, convertedPt,
                                                                     m_radius, pt.GetColor());

                          m_handlesCache[cacheIndex].second++;
                          if (m_handlesCache[cacheIndex].second >= m_handlesCache[cacheIndex].first->GetPointsCount())
                          {
                            cacheIndex++;
                            if (cacheIndex >= m_handlesCache.size())
                            {
                              m_dataRequestFn(kAveragePointsCount);
                              m_waitForRenderData = true;
                              return;
                            }
                          }
                        }
                      });
    m_needUpdate = false;

    // We have asked the backend for an additional buffer pack, wait until it arrives.
    if (m_waitForRenderData)
      return;
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

void StreetPixelRenderer::Clear()
{
  m_tileBuckets.clear();
  ClearRenderData();
}
}  // namespace df
