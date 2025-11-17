#ifndef RAINCOM_COM_MAP_LANE_H_
#define RAINCOM_COM_MAP_LANE_H_

#include <memory>
#include <raincom_msgs/relative_map.pb.h>

namespace raincom
{
namespace com
{
namespace map
{

struct LaneSegment
{
    LaneSegment() = default;
    LaneSegment(float start_s, float end_s, std::shared_ptr<const msgs::Lane> lane)
        : start_s(start_s), end_s(end_s), lane(lane)
    {
    }
    inline float length() const { return end_s - start_s; }

    float start_s;
    float end_s;
    std::shared_ptr<const msgs::Lane> lane;
}; // struct LaneSegment

} // namespace map
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_MAP_LANE_H_
