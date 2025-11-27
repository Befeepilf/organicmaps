#pragma once

#include "drape/color.hpp"
#include "geometry/point2d.hpp"

#include <cstdint>

namespace df
{
class StreetPixel
{
public:
  StreetPixel() = default;

  bool IsExplored() const;
  void SetExplored(bool explored);

  dp::Color const GetColor() const;
  std::int64_t GetPixelId() const;
  m2::PointD const GetPoint() const;

private:
  std::int64_t m_pixelId;
};
}  // namespace df