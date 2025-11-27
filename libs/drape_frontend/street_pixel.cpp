#include "base/assert.hpp"
#include "base/math.hpp"

#include "drape_frontend/street_pixel.hpp"

#include "drape/color.hpp"

#include "geometry/mercator.hpp"
#include "geometry/point2d.hpp"

#include "map/street_pixels_manager.hpp"

#include <healpix_base.h>
#include <cmath>
#include <cstdint>

namespace df
{
std::int64_t StreetPixel::GetPixelId() const { return m_pixelId & 0x7FFFFFFFFFFFFFFF; }

bool StreetPixel::IsExplored() const { return m_pixelId & 0x8000000000000000; }

void StreetPixel::SetExplored(bool explored) { m_pixelId |= (explored ? 0x8000000000000000 : 0); }

dp::Color const StreetPixel::GetColor() const
{
  if (IsExplored())
    return dp::Color::Green();
  return dp::Color::Red();
}

m2::PointD const StreetPixel::GetPoint() const
{
  pointing const & ang = hp::GetHealpixBase().pix2ang(GetPixelId());
  double const latDeg = math::RadToDeg(M_PI_2 - ang.theta);
  double const lonDeg = math::RadToDeg(ang.phi);
  return mercator::FromLatLon(latDeg, lonDeg);
}
}  // namespace df