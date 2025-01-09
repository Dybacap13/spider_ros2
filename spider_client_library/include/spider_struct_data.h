#pragma once

#include <math.h>

#include <iostream>
#include <string>
#include <vector>

namespace spider_client_library {

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
  std::string name;
  double coxa;
  double femur;
  double tibia;
};

struct Trig {
  double sine;
  double cosine;
};

struct RosParametrs {
  std::vector<double> coxa_to_center_x;
  std::vector<double> coxa_to_center_y;
  std::vector<double> init_coxa_angle;
  double coxa_length;
  double femur_length;
  double tibia_length;
  double tarsus_length;
  int number_of_legs;
};
}  // namespace spider_client_library