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
std::vector<std::string> names_leg_ik = {"rr", "rm", "rf", "lr", "lm", "lf"};
static const double PI = atan(1.0) * 4.0;
class SpiderIk {
 public:
  SpiderIk(RosParametrs ros_parametrs_);
  Eigen::Matrix<double, 4, 4> transformationDenaviteHartenberg(double a,
                                                               double alpha,
                                                               double d,
                                                               double tetta);
  std::vector<TransformStamped> getPositionFoot();

  std::vector<spider_client_library::SpiderData> getJointLeg(
      TransformStamped offset,
      std::vector<spider_client_library::JointLeg> joints);

  SpiderData IK(const std::vector<TransformStamped> feet,
                const TransformStamped body, bool state);

 private:
  SpiderData ikCalculeterOwn(const std::vector<TransformStamped> feet);
  std::vector<TransformStamped> coordFeetFromCoxa(std::vector<JointLeg> joints);
  std::vector<TransformStamped> foot_current;
  std::vector<JointLeg> angle_joint_leg_current;

  TransformStamped rotationMatrixToPRY(
      Eigen::Matrix<double, 3, 3> rotationMatrix);
  Position getCoordinateFromTDH(Eigen::Matrix<double, 4, 4> matrix);
  Trig getSinCos(double angle_rad);
  TransformStamped getTargetBodyFromOffset(TransformStamped body,
                                           TransformStamped offset);
  TransformStamped calculateDistanseOffsetBody(TransformStamped body_update,
                                               TransformStamped body_current);
  std::vector<TransformStamped> coordFeetFromBody(TransformStamped body);
  TransformStamped calculateRotaryBodyZ(TransformStamped body_target,
                                        TransformStamped body_current);
  RosParametrs ros_parametrs;
  std::vector<TransformStamped> coordFeetFromBody(TransformStamped body,
                                                  std::vector<JointLeg> joints);

  TransformStamped body_current;
};
}  // namespace spider_client_library

#endif