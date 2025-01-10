
#include <spider_client_imitation.h>
#include <spider_client_interface.h>
#include <control_msgs/action/joint_trajectory.hpp>

#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spider_msgs/srv/ik.hpp>
using namespace spider_client_library;
namespace spider_driver {

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
  void getRosParam();
  void createService();
  void createSubscriber();
  void createClient();
  void createAction();
  void createInterfaceClient(std::vector<double> initial_pose);

  std::shared_ptr<rclcpp::Node> spider_control_node;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Client<spider_msgs::srv::IK>::SharedPtr ik_calculator_client;

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