#pragma once

#include "drape/color.hpp"
#include "geometry/point2d.hpp"

#include <cstdint>

namespace df
{
class StreetPixelPoint : public m2::PointD
{
public:
  StreetPixelPoint();
  StreetPixelPoint(int64_t pixelId, double x, double y, bool explored_);

  dp::Color const GetColor() const;
  int64_t GetPixelId() const;

  bool explored;

private:
  int64_t m_pixelId;
};
}  // namespace df
