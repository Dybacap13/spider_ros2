#ifndef SPIDER_CLIENT_LIBRARY_H_
#define SPIDER_CLIENT_LIBRARY_H_

#include <math.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>
#include <string>
#include <vector>

#include "spider_struct_data.h"

namespace spider_client_library {
static const double PI = atan(1.0) * 4.0;
class SpiderIk {
 public:
  SpiderIk(RosParametrs ros_parametrs_);
  Eigen::Matrix<double, 4, 4> transformationDenaviteHartenberg(double a,
                                                               double alpha,
                                                               double d,
                                                               double tetta);
  std::vector<TransformStamped> getPositionFoot();

  std::vector<JointLeg> IK(const std::vector<TransformStamped> feet,
                           const TransformStamped body, bool state);

 private:
  std::vector<TransformStamped> foot_current;
  std::vector<JointLeg> angle_joint_leg_current;
  Position getCoordinateFromTDH(Eigen::Matrix<double, 4, 4> matrix);
  Trig getSinCos(double angle_rad);
  TransformStamped calculateDistanseOffsetBody(TransformStamped body_update,
                                               TransformStamped body_current);
  std::vector<TransformStamped> coordFeetFromBody(TransformStamped body);
  TransformStamped calculateRotaryBodyZ(TransformStamped body_target,
                                        TransformStamped body_current);
  RosParametrs ros_parametrs;
  std::vector<TransformStamped> coordFeetFromBody(TransformStamped body,
                                                  std::vector<JointLeg> joints);
};
}  // namespace spider_client_library

#endif