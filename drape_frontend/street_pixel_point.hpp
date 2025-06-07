#pragma once

#include "drape/color.hpp"
#include "geometry/point2d.hpp"

#include <cstdint>

namespace df
{
struct StreetPixelPoint
{
  m2::PointD m_point;
  dp::Color m_color;
  int64_t m_pixelId;
};
}  // namespace df
