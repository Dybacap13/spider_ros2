
#include <controllers_gazebo.hpp>
using namespace std::chrono_literals;
namespace spider_gazebo_controllers {
GazeboControllers::GazeboControllers(rclcpp::NodeOptions options)
    : Node("spider_gazebo_controllers",
           options.allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true)) {
  RCLCPP_INFO_STREAM(this->get_logger(), "GazeboControllers : READY");

  createSubscriber();
  createMapPublisher();

  // async_thread_ = std::make_shared<std::thread>(
  //     &GazeboControllers::publishGazeboControllers, this);
}

void GazeboControllers::createSubscriber() {
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        jointStatesCallback(msg);
      });
}
void GazeboControllers::jointStatesCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  joint_current = msg;
  publishGazeboControllers();
}

void GazeboControllers::createMapPublisher() {
  for (auto name : names_coxa) {
    publisher_joints_to_controller.insert(
        {name, create_publisher<std_msgs::msg::Float64MultiArray>(
                   "/" + name + "_controller/commands", 10)});
  }

  for (auto name : names_femur) {
    publisher_joints_to_controller.insert(
        {name, create_publisher<std_msgs::msg::Float64MultiArray>(
                   "/" + name + "_controller/commands", 10)});
  }

  for (auto name : names_tibia) {
    publisher_joints_to_controller.insert(
        {name, create_publisher<std_msgs::msg::Float64MultiArray>(
                   "/" + name + "_controller/commands", 10)});
  }
}

void GazeboControllers::publishGazeboControllers() {
  for (size_t index = 0; index < publisher_joints_to_controller.size();
       index++) {
    std_msgs::msg::Float64MultiArray msg;

    msg.data.push_back(joint_current->position[index]);
    publisher_joints_to_controller[joint_current->name[index]]->publish(msg);
  }
  std::this_thread::sleep_for(500ms);
}
}  // namespace spider_gazebo_controllers

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(spider_gazebo_controllers::GazeboControllers)
