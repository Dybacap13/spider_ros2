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

struct SpiderData {
  std::vector<std::string> names;
  std::vector<JointLeg> legs;
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

struct TimeJointTrajectory {
  std::int32_t sec;
  std::uint32_t nanosec;
};
struct HeaderJointTrajectory {
  TimeJointTrajectory stamp;
  std::string frame_id;
};

struct DurationJoint {
  std::int32_t sec;
  std::uint32_t nanosec;
};

struct JointTrajectoryPoint {
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
  std::vector<double> effort;
  DurationJoint time_from_start;
};

struct JointTrajectory {
  HeaderJointTrajectory header;
  std::vector<std::string> joint_names;
  std::vector<JointTrajectoryPoint> points;
};

}  // namespace spider_client_library