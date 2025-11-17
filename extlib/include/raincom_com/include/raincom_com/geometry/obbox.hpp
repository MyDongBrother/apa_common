#ifndef RAINCOM_COM_GEOMETRY_OBBOX_H_
#define RAINCOM_COM_GEOMETRY_OBBOX_H_

#include "raincom_com/geometry/point2d.hpp"
#include <algorithm>
#include <array>
#include <iterator>
#include <vector>

namespace raincom
{
namespace com
{
namespace geometry
{

class OBBox2d
{
    using BoxPoint = Point2d<float>;

  public:
    OBBox2d(float heading, const BoxPoint &c1, const BoxPoint &c2, const BoxPoint &c3,
            const BoxPoint &c4)
        : heading_(heading)
    {
        corners_.at(0) = c1;
        corners_.at(1) = c2;
        corners_.at(2) = c3;
        corners_.at(3) = c4;
    }
    OBBox2d(float heading, const std::vector<BoxPoint> &corners) : heading_(heading)
    {
        std::copy(corners.begin(), corners.end(), std::back_inserter(corners_));
    }
    const float &heading() const { return heading_; }
    const std::array<BoxPoint, 4> &corners() const { return corners_; }

  private:
    std::array<BoxPoint, 4> corners_;
    float heading_;
}; // class OBBox

} // namespace geometry
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_GEOMETRY_OBBOX_H_
