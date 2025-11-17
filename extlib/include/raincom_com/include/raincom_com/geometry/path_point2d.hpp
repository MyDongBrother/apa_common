#ifndef RAINCOM_COM_GEOMETRY_PATH_POINT2D_H_
#define RAINCOM_COM_GEOMETRY_PATH_POINT2D_H_

#include "raincom_com/geometry/point2d.hpp"

namespace raincom
{
namespace com
{
namespace geometry
{

template <typename T>
class PathPoint2d : public Point2d<T>
{
  public:
    PathPoint2d() : heading_(0.0) {}
    PathPoint2d(T x, T y, T heading) : Point2d<T>(x, y), heading_(heading) {}

    inline const T &heading() const { return heading_; }
    inline void set_heading(const T &heading) { heading_ = heading; }

  private:
    T heading_;
}; // class PathPoint2d

} // namespace geometry
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_GEOMETRY_PATH_POINT2D_H_
