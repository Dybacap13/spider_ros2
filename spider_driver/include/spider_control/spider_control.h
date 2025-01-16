
#include <spider_client_imitation.h>
#include <spider_client_interface.h>

#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <control_msgs/action/joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spider_msgs/msg/joints.hpp>
#include <spider_msgs/srv/ik.hpp>

using namespace spider_client_library;
namespace spider_driver {

std::vector<std::string> joint_names_joint_state = {
    "joint_coxa_rr"
    "joint_femur_rr",
    "joint_tibia_rr",
    "joint_coxa_rm",
    "joint_femur_rm",
    "joint_tibia_lf",
    "joint_coxa_lf",
    "joint_tibia_lm",
    "joint_tibia_rm",
    "joint_femur_lm",
    "joint_femur_lf",
    "joint_coxa_lr",
    "joint_tibia_lr",
    "joint_femur_lr",
    "joint_coxa_rf",
    "joint_tibia_rf",
    "joint_coxa_lm",
    "joint_femur_rf"};

class SpiderControl {
 public:
  SpiderControl(std::vector<double> initial_pose);
  ~SpiderControl();
  std::vector<double> getJointData();
  using JointTrajectoryMsg = control_msgs::action::JointTrajectory;
  using JointTrajectoryGoal =
      rclcpp_action::ServerGoalHandle<JointTrajectoryMsg>;

 private:
  void spinThreadKHI();
  void createAction();
  void createInterfaceClient(std::vector<double> initial_pose);

  std::shared_ptr<rclcpp::Node> spider_control_node;

  std::shared_ptr<spider_client_library::SpiderClientInterface>
      spider_interface;

  // action
  rclcpp_action::GoalResponse handleGoalTrajectory(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const JointTrajectoryMsg::Goal> /*goal*/);
  rclcpp_action::CancelResponse handleCancelTrajectory(
      const std::shared_ptr<JointTrajectoryGoal> goal_handle);
  void handleAcceptedTrajectory(
      const std::shared_ptr<JointTrajectoryGoal> goal_handle);
  void executeTrajectory(
      const std::shared_ptr<JointTrajectoryGoal> goal_handle);

  // action param
  rclcpp_action::Server<JointTrajectoryMsg>::SharedPtr action_move;
  std::shared_ptr<std::thread> async_thread_;

  std::string action_name_trajectory = "/spider_control/move_by_trajectory";

  std::mutex traj_mutex;
  std::mutex stop_mutex;
  std::mutex movement_state_mutex;
  std::mutex state_robot_mode_mutex;
};

}  // namespace spider_driver
