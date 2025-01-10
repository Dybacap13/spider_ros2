#include <spider_ik_server/spider_ik_server.h>

namespace spider_ik {

IkServers::IkServers(rclcpp::NodeOptions options)
    : Node("spider_ik",
           options.allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true)) {
  RCLCPP_INFO_STREAM(this->get_logger(), "IkServers : READY");
  createService();
  getRosParam();
  createActionClient();
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "END" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
}

void IkServers::createActionClient() {
  spider_control_action_client_ =
      rclcpp_action::create_client<control_msgs::action::JointTrajectory>(
          this, "/spider_control/move_by_trajectory");
}

void IkServers::resultActionClient(
    const rclcpp_action::ClientGoalHandle<
        control_msgs::action::JointTrajectory>::WrappedResult& result) {
  RCLCPP_INFO_STREAM(this->get_logger(), "SUCCEEDED ACTION");
}

void IkServers::getRosParam() {
  try {
    readRosParam("NUMBER_OF_LEGS", NUMBER_OF_LEGS);
    readRosParam("COXA_LENGTH", COXA_LENGTH);
    readRosParam("FEMUR_LENGTH", FEMUR_LENGTH);
    readRosParam("TIBIA_LENGTH", TIBIA_LENGTH);
    readRosParam("TARSUS_LENGTH", TARSUS_LENGTH);
    readRosParam("COXA_TO_CENTER_X", COXA_TO_CENTER_X);
    readRosParam("COXA_TO_CENTER_Y", COXA_TO_CENTER_Y);
    readRosParam("INIT_COXA_ANGLE", INIT_COXA_ANGLE);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Get param failed");
    RCLCPP_ERROR(this->get_logger(), e.what());
  }
}
template <typename T>
void IkServers::readRosParam(const std::string param_name, T& param_value) {
  if (!this->get_parameter(param_name, param_value)) {
    RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Parameter \"" << param_name << "\" didn' find in Parameter Server.");
    throw(std::runtime_error(" Error read parametrs"));
  }
}

void IkServers::createService() {
  service_ik = create_service<spider_msgs::srv::IK>(
      "spider/ik_calculator",
      std::bind(&IkServers::getCalculateIk, this, std::placeholders::_1,
                std::placeholders::_2));
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "SERVIS" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
}
void IkServers::getCalculateIk(
    const std::shared_ptr<spider_msgs::srv::IK::Request> request,
    std::shared_ptr<spider_msgs::srv::IK::Response> response) {
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "request" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;
  std::cout << "-" << std::endl;

  // auto fake = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  trajectory_msgs::msg::JointTrajectoryPoint fake_point;
  std::vector<double> joints_(18, 0);
  fake_point.positions = joints_;
  // fake->points.emplace_back(fake_point);

  auto goal_msg = control_msgs::action::JointTrajectory::Goal();

  goal_msg.trajectory.points.emplace_back(fake_point);
  auto send_goal_options = rclcpp_action::Client<
      control_msgs::action::JointTrajectory>::SendGoalOptions();
  send_goal_options.result_callback =
      std::bind(&IkServers::resultActionClient, this, std::placeholders::_1);
  auto goal_handle_future = spider_control_action_client_->async_send_goal(
      goal_msg, send_goal_options);

  RCLCPP_INFO(this->get_logger(), "Accepted new action goal");
}
}  // namespace spider_ik

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(spider_ik::IkServers)
