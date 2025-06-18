#include "drape_frontend/street_pixel_point.hpp"

#include "drape/color.hpp"

#include "geometry/point2d.hpp"

#include <cstdint>

namespace df
{
StreetPixelPoint::StreetPixelPoint()
  : m2::PointD(0, 0)
  , explored(false)
  , m_pixelId()
{}

StreetPixelPoint::StreetPixelPoint(int64_t pixelId, double x, double y, bool explored_)
  : m2::PointD(x, y)
  , explored(explored_)
  , m_pixelId(pixelId)
{}

std::int64_t StreetPixelPoint::GetPixelId() const { return m_pixelId; }

dp::Color const StreetPixelPoint::GetColor() const
{
  if (explored)
    return dp::Color::Green();
  return dp::Color::Red();
}
}  // namespace df
