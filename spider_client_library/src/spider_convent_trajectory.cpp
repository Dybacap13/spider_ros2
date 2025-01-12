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

//  "joint_coxa_rr"
//   "joint_femur_rr",
//   "joint_tibia_rr",

//   "joint_coxa_rm",
//   "joint_femur_rm",
//   "joint_tibia_lf",

//   "joint_coxa_lf",
//   "joint_tibia_lm",
//   "joint_tibia_rm",

//   "joint_femur_lm",
//   "joint_femur_lf",
//   "joint_coxa_lr",

//   "joint_tibia_lr",
//   "joint_femur_lr",
//   "joint_coxa_rf",

//   "joint_tibia_rf",
//   "joint_coxa_lm",
//   "joint_femur_rf"};
JointTrajectory SpiderConventTrajectory::conventFromJointsLegsToTragectoryMsg(
    std::vector<SpiderData> joint_points,
    std::map<std::string, double> joints_map,
    std::vector<std::string> names_vector) {
  JointTrajectory trajectory;
  std::cout << "AAAAAAAAAAAAAAA" << std::endl;
  std::cout << joint_points.size() << std::endl;
  std::cout << joint_points[0].legs.size() << std::endl;
  for (int i = 0; i < joint_points[0].legs.size(); i++) {
    std::cout << joint_points[0].legs[i].coxa << std::endl;
    std::cout << joint_points[0].legs[i].femur << std::endl;
    std::cout << joint_points[0].legs[i].tibia << std::endl;
    std::cout << "----" << std::endl;
  }

  for (size_t index = 0; index < joint_points.size(); index++) {
    std::vector<double> joints_position;
    for (size_t leg = 0; leg < joint_points[index].legs.size(); leg++) {
      joints_position.emplace_back(joint_points[index].legs[leg].coxa);
      joints_position.emplace_back(joint_points[index].legs[leg].femur);
      joints_position.emplace_back(joint_points[index].legs[leg].tibia);
    }

    // //   "joint_coxa_rm",
    // //   "joint_femur_rm",
    // //   "joint_tibia_lf",
    // joints_position[4] = joint_points[index].legs[1].coxa;
    // joints_position[5] = joint_points[index].legs[1].femur;
    // joints_position[6] = joint_points[index].legs[5].tibia;

    // //   "joint_coxa_lf",
    // //   "joint_tibia_lm",
    // //   "joint_tibia_rm",
    // joints_position[7] = joint_points[index].legs[5].coxa;
    // joints_position[8] = joint_points[index].legs[4].tibia;
    // joints_position[9] = joint_points[index].legs[1].tibia;

    // //   "joint_femur_lm",
    // //   "joint_femur_lf",
    // //   "joint_coxa_lr",

    // joints_position[10] = joint_points[index].legs[4].femur;
    // joints_position[11] = joint_points[index].legs[5].femur;
    // joints_position[12] = joint_points[index].legs[3].coxa;

    // //   "joint_tibia_lr",
    // //   "joint_femur_lr",
    // //   "joint_coxa_rf",

    // joints_position[13] = joint_points[index].legs[3].tibia;
    // joints_position[14] = joint_points[index].legs[3].femur;
    // joints_position[15] = joint_points[index].legs[2].coxa;

    // //   "joint_tibia_rf",
    // //   "joint_coxa_lm",
    // //   "joint_femur_rf"};

    // joints_position[13] = joint_points[index].legs[2].tibia;
    // joints_position[14] = joint_points[index].legs[4].coxa;
    // joints_position[15] = joint_points[index].legs[2].femur;
    JointTrajectoryPoint point;
    point.positions = (joints_position);
    trajectory.points.emplace_back(point);
  }
  return trajectory;
}
}  // namespace spider_client_library