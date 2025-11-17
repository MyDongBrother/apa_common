#ifndef RAINCOM_COM_POSE_H_
#define RAINCOM_COM_POSE_H_

#include <Eigen/Geometry>

namespace raincom
{
namespace com
{
namespace geometry
{

void heading_to_quaternion(const float heading, Eigen::Quaternionf &qtn);
void quaternion_to_heading(const Eigen::Quaternionf &qtn, float &heading);

} // namespace geometry
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_POSE_H_
