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
  createSubscriber();

  ik_solver = std::make_shared<spider_client_library::SpiderIk>(ik_param);
  conventer =
      std::make_shared<spider_client_library::SpiderConventTrajectory>();

  gait_solver =
      std::make_shared<spider_client_library::SpiderGaitGenerator>(gait_param);
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
    readRosParam("NUMBER_OF_LEGS", ik_param.number_of_legs);
    readRosParam("COXA_LENGTH", ik_param.coxa_length);
    readRosParam("FEMUR_LENGTH", ik_param.femur_length);
    readRosParam("TIBIA_LENGTH", ik_param.tibia_length);
    readRosParam("TARSUS_LENGTH", ik_param.tarsus_length);
    readRosParam("COXA_TO_CENTER_X", ik_param.coxa_to_center_x);
    readRosParam("COXA_TO_CENTER_Y", ik_param.coxa_to_center_y);
    readRosParam("INIT_COXA_ANGLE", ik_param.init_coxa_angle);
    readRosParam("STEP_LENGTH", gait_param.step_lenght);
    readRosParam("STEP_HEIGHT", gait_param.step_height);
    readRosParam("NUMBER_POINTS", gait_param.number_points);

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
      "spider/ik_calculator_relative_coxa",
      std::bind(&IkServers::getCalculateIk, this, std::placeholders::_1,
                std::placeholders::_2));

  service_gait = create_service<spider_msgs::srv::IK>(
      "spider/gait_generator",
      std::bind(&IkServers::getCalculateGait, this, std::placeholders::_1,
                std::placeholders::_2));
}
void IkServers::getCalculateIk(
    const std::shared_ptr<spider_msgs::srv::IK::Request> request,
    std::shared_ptr<spider_msgs::srv::IK::Response> response) {
  std::map<std::string, double> joint_with_names;
  for (size_t index = 0; index < joint_current->name.size(); index++) {
    joint_with_names.insert(
        {joint_current->name[index], joint_current->position[index]});
  }
  spider_client_library::TransformStamped offset;
  offset.position.x = request->offset.linear.x;
  offset.position.y = request->offset.linear.y;
  offset.position.z = request->offset.linear.z;
  offset.orientation.pitch = request->offset.angular.x;
  offset.orientation.roll = request->offset.angular.y;
  offset.orientation.yaw = request->offset.angular.z;

  auto joint_leg = conventer->conventFromMapJontsToLegJoints(joint_with_names);
  auto spider_data = ik_solver->getJointLeg(offset, joint_leg);

  auto points = conventer->conventFromJointsLegsToTragectoryMsg(
      spider_data, joint_with_names, joint_current->name);

  auto goal_msg = control_msgs::action::JointTrajectory::Goal();
  for (size_t index = 0; index < points.points.size(); index++) {
    trajectory_msgs::msg::JointTrajectoryPoint point_tj;
    point_tj.positions = points.points[index].positions;
    goal_msg.trajectory.points.emplace_back(point_tj);
  }
  // MUTEX

  auto send_goal_options = rclcpp_action::Client<
      control_msgs::action::JointTrajectory>::SendGoalOptions();
  send_goal_options.result_callback =
      std::bind(&IkServers::resultActionClient, this, std::placeholders::_1);
  auto goal_handle_future = spider_control_action_client_->async_send_goal(
      goal_msg, send_goal_options);

  RCLCPP_INFO(this->get_logger(), "Accepted new action goal");
}
void IkServers::createSubscriber() {
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        jointStatesCallback(msg);
      });
}
void IkServers::jointStatesCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  joint_current = msg;
}

void IkServers::getCalculateGait(
    const std::shared_ptr<spider_msgs::srv::IK::Request> request,
    std::shared_ptr<spider_msgs::srv::IK::Response> response) {
  std::map<std::string, double> joint_with_names;
  for (size_t index = 0; index < joint_current->name.size(); index++) {
    joint_with_names.insert(
        {joint_current->name[index], joint_current->position[index]});
  }
  spider_client_library::TransformStamped offset;
  offset.position.x = request->offset.linear.x;
  offset.position.y = request->offset.linear.y;
  offset.position.z = request->offset.linear.z;
  offset.orientation.pitch = request->offset.angular.x;
  offset.orientation.roll = request->offset.angular.y;
  offset.orientation.yaw = request->offset.angular.z;

  auto joint_leg = conventer->conventFromMapJontsToLegJoints(joint_with_names);
  auto joint_position = ik_solver->coordFeetFromCoxa(joint_leg);
  std::vector<spider_client_library::SpiderData> spider_data_vector;
  for (size_t point = 0; point < gait_param.number_points - 1; point++) {
    std::cout << "POINT -----" << std::endl;
    auto joint_position_point_gait = gait_solver->getGaitPoints(joint_position);
    auto spider_data = ik_solver->ikCalculeterOwn(joint_position_point_gait);
    for (int i = 0; i < 1; i++) {
      std::cout << "coxa = " << spider_data.legs[i].coxa << std::endl;
      std::cout << "femur = " << spider_data.legs[i].femur << std::endl;
      std::cout << "tibia = " << spider_data.legs[i].tibia << std::endl;
    }
    spider_data_vector.emplace_back(spider_data);
  }

  auto points = conventer->conventFromJointsLegsToTragectoryMsg(
      spider_data_vector, joint_with_names, joint_current->name);

  auto goal_msg = control_msgs::action::JointTrajectory::Goal();
  for (size_t index = 0; index < points.points.size(); index++) {
    trajectory_msgs::msg::JointTrajectoryPoint point_tj;
    point_tj.positions = points.points[index].positions;
    goal_msg.trajectory.points.emplace_back(point_tj);
  }
  // MUTEX

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
