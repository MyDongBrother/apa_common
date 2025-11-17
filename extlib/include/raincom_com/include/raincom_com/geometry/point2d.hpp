#ifndef RAINCOM_COM_GEOMETRY_POINT2D_H_
#define RAINCOM_COM_GEOMETRY_POINT2D_H_

#include <cmath>

namespace raincom
{
namespace com
{
namespace geometry
{

template <typename T>
class Point2d
{
  public:
    Point2d() : x_(0.0), y_(0.0) {}
    Point2d(T x, T y) : x_(x), y_(y) {}
    inline const T &x() const { return x_; }
    inline const T &y() const { return y_; }
    inline void set_x(const T &x) { x_ = x; }
    inline void set_y(const T &y) { y_ = y; }
    inline T distance(const Point2d<T> &other);

  protected:
    T x_;
    T y_;
}; // class Point2d

template <typename T>
inline T Point2d<T>::distance(const Point2d<T> &other)
{
    return std::sqrt(std::pow(x_ - other.x_, 2) + std::pow(y_ - other.y_, 2));
}

} // namespace geometry
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_GEOMETRY_POINT2D_H_
