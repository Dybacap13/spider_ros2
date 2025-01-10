
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <spider_ik.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>
#include <spider_msgs/msg/joints.hpp>
#include <spider_msgs/srv/ik.hpp>
using namespace std::chrono_literals;
namespace spider_ik {

class IkServers : public rclcpp::Node {
 public:
  IkServers(rclcpp::NodeOptions options);

 private:
  void spinThreadKHI();
  void getRosParam();
  void createService();

  std::shared_ptr<std::thread> async_thread_;
  // std::shared_ptr<rclcpp::Node> spider_control_node;
  std::shared_ptr<spider_client_library::SpiderIk> ik_solver;

  // parametrs
  std::vector<double> COXA_TO_CENTER_X, COXA_TO_CENTER_Y;
  std::vector<double> INIT_COXA_ANGLE;
  // std::vector<double> INIT_FOOT_POS_X, INIT_FOOT_POS_Y, INIT_FOOT_POS_Z;
  double COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH;
  int NUMBER_OF_LEGS;
  template <typename T>
  void readRosParam(const std::string param_name, T& param_value);

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
};

}  // namespace spider_ik