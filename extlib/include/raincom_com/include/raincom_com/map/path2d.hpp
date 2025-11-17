#ifndef RAINCOM_COM_MAP_PATH2D_H_
#define RAINCOM_COM_MAP_PATH2D_H_

#include "raincom_com/map/path_point2d.hpp"
#include "raincom_com/map/lane.hpp"
#include <vector>

namespace raincom
{
namespace com
{
namespace map
{

class Path2d
{
  public:
    Path2d();

  private:
    std::vector<PathPoint2d<float>> points_;
    std::vector<LaneSegment> lane_segments_;
}; // class Path2d

} // namespace map
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_MAP_PATH2D_H_
