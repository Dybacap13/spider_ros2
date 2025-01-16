#include <spider_convent_trajectory.h>

namespace spider_client_library {

SpiderConventTrajectory::SpiderConventTrajectory() {
  std::cout << "SpiderConventTrajectory ready" << std::endl;
}

std::vector<JointLeg> SpiderConventTrajectory::conventFromMapJontsToLegJoints(
    std::map<std::string, double> joints) {
  std::vector<JointLeg> joint_legs;
  for (size_t index_leg = 0; index_leg < names_leg.size(); index_leg++) {
    JointLeg joint_leg;
    if (joints.find(names_coxa[index_leg]) == joints.end()) {
      throw std::logic_error("SpiderConventTrajectory.key not found");
    }
    if (joints.find(names_femur[index_leg]) == joints.end()) {
      throw std::logic_error("SpiderConventTrajectory.key not found");
    }

    if (joints.find(names_tibia[index_leg]) == joints.end()) {
      throw std::logic_error("SpiderConventTrajectory.key not found");
    }

    joint_leg.name = names_leg[index_leg];
    joint_leg.coxa = joints[names_coxa[index_leg]];
    joint_leg.femur = joints[names_femur[index_leg]];
    joint_leg.tibia = joints[names_tibia[index_leg]];
    joint_legs.emplace_back(joint_leg);
  }
  return joint_legs;
}

JointTrajectory SpiderConventTrajectory::conventFromJointsLegsToTragectoryMsg(
    std::vector<SpiderData> joint_points,
    std::map<std::string, double> joints_map,
    std::vector<std::string> names_vector) {
  JointTrajectory trajectory;
  for (size_t index = 0; index < joint_points.size(); index++) {
    std::vector<double> joints_position;
    for (size_t leg = 0; leg < joint_points[index].legs.size(); leg++) {
      joints_position.emplace_back(joint_points[index].legs[leg].coxa);
      joints_position.emplace_back(joint_points[index].legs[leg].femur);
      joints_position.emplace_back(joint_points[index].legs[leg].tibia);
    }
    JointTrajectoryPoint point;
    point.positions = (joints_position);
    trajectory.points.emplace_back(point);
  }
  return trajectory;
}
}  // namespace spider_client_library