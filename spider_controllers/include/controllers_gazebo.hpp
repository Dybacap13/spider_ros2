
#include <math.h>

#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <vector>
namespace spider_gazebo {
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

class GazeboControllers : public rclcpp::Node {
 public:
  GazeboControllers(rclcpp::NodeOptions options);

 private:
  void createSubscriber();
  void createMapPublisher();
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publishGazeboControllers();

  std::shared_ptr<std::thread> async_thread_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;

  sensor_msgs::msg::JointState::SharedPtr joint_current;

  std::map<std::string,
           rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr>
      publisher_joints_to_controller;
};
}  // namespace spider_gazebo