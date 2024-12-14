#ifndef SPIDER_CLIENT_LIBRARY_H_
#define SPIDER_CLIENT_LIBRARY_H_
#include <iostream.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <string>
#include <vector>

namespace spider_client_library {
static const double PI = atan(1.0) * 4.0;
struct Position {
  double x;
  double y;
  double z;
};

struct OrientationRPY {
  double roll;
  double pitch;
  double yaw;
};

struct TransformStamped {
  Position position;
  OrientationRPY orientation;
};

struct JointLeg {
  double coxa;
  double femur;
  double tibia;
};

struct Trig {
  double sine;
  double cosine;
};

class SpiderIk {
 public:
  SpiderIk();
  Eigen::Matrix<double, 4, 4> transformationDenaviteHartenberg(double a,
                                                               double alpha,
                                                               double d,
                                                               double tetta);
  std::vector<TransformStamped> getPositionFoot();

  std::vector<JointLeg> IK(const std::vector<TransformStamped> feet,
                           const TransformStamped body, bool state);

 private:
  TransformStamped body_current;
  std::vector<TransformStamped> foot_current;
  std::vector<JointLeg> angle_joint_leg_current;

  Trig getSinCos(double angle_rad);
  void calculateIK(TransformStamped body_update);
  std::vector<TransformStamped> coordFeetFromBody(TransformStamped body);

  std::vector<double> COXA_TO_CENTER_X, COXA_TO_CENTER_Y;
  std::vector<double> INIT_COXA_ANGLE;
  std::vector<double> INIT_FOOT_POS_X, INIT_FOOT_POS_Y, INIT_FOOT_POS_Z;
  double COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH;
  int NUMBER_OF_LEGS;
};
}  // namespace spider_client_library

#endif