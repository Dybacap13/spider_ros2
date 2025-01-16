#include <math.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "spider_struct_data.h"

namespace spider_client_library {
std::vector<std::string> names_coxa = {"joint_coxa_rr", "joint_coxa_rm",
                                       "joint_coxa_rf", "joint_coxa_lr",
                                       "joint_coxa_lm", "joint_coxa_lf"};

std::vector<std::string> names_femur = {"joint_femur_rr", "joint_femur_rm",
                                        "joint_femur_rf", "joint_femur_lr",
                                        "joint_femur_lm", "joint_femur_lf"};

std::vector<std::string> names_tibia = {"joint_tibia_rr", "joint_tibia_rm",
                                        "joint_tibia_rf", "joint_tibia_lr",
                                        "joint_tibia_lm", "joint_tibia_lf"};

std::vector<std::string> names_leg = {"rr", "rm", "rf", "lr", "lm", "lf"};

class SpiderConventTrajectory {
 public:
  SpiderConventTrajectory();

  std::vector<JointLeg> conventFromMapJontsToLegJoints(
      std::map<std::string, double> joints);

  JointTrajectory conventFromJointsLegsToTragectoryMsg(
      std::vector<SpiderData> joint_points,
      std::map<std::string, double> joints,
      std::vector<std::string> names_vector);
};
}  // namespace spider_client_library