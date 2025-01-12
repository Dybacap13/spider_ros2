
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <spider_convent_trajectory.h>
#include <spider_ik.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <spider_msgs/msg/joints.hpp>
#include <spider_msgs/srv/ik.hpp>
using namespace std::chrono_literals;
namespace spider_ik {

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

class IkServers : public rclcpp::Node {
 public:
  IkServers(rclcpp::NodeOptions options);

 private:
  void spinThreadKHI();
  void getRosParam();
  void createService();
  void createSubscriber();

  std::shared_ptr<std::thread> async_thread_;
  // std::shared_ptr<rclcpp::Node> spider_control_node;
  std::shared_ptr<spider_client_library::SpiderIk> ik_solver;
  std::shared_ptr<spider_client_library::SpiderConventTrajectory> conventer;

  // parametrs
  std::vector<double> COXA_TO_CENTER_X, COXA_TO_CENTER_Y;
  std::vector<double> INIT_COXA_ANGLE;
  // std::vector<double> INIT_FOOT_POS_X, INIT_FOOT_POS_Y, INIT_FOOT_POS_Z;
  double COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH;
  int NUMBER_OF_LEGS;
  template <typename T>
  void readRosParam(const std::string param_name, T& param_value);
  spider_client_library::RosParametrs ros_param;

  // servers
  rclcpp::Service<spider_msgs::srv::IK>::SharedPtr service_ik;
  void getCalculateIk(
      const std::shared_ptr<spider_msgs::srv::IK::Request> request,
      std::shared_ptr<spider_msgs::srv::IK::Response> response);

  // action
  rclcpp_action::Client<control_msgs::action::JointTrajectory>::SharedPtr
      spider_control_action_client_;
  std::mutex mutex_action_client;
  void createActionClient();
  void resultActionClient(
      const rclcpp_action::ClientGoalHandle<
          control_msgs::action::JointTrajectory>::WrappedResult& result);

  // sub
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  sensor_msgs::msg::JointState::SharedPtr joint_current;
};

}  // namespace spider_ik