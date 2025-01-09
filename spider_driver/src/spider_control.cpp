
#include <spider_control/spider_control.h>
using std::placeholders::_1;
using namespace std::chrono_literals;
namespace spider_driver {

SpiderControl::SpiderControl(std::vector<double> initial_pose)
    : spider_control_node(
          std::make_shared<rclcpp::Node>("spider_control_node")) {
  createSubscriber();
  createClient();
  createInterfaceClient(initial_pose);
  async_thread_ =
      std::make_shared<std::thread>(&SpiderControl::spinThreadKHI, this);
}

SpiderControl::~SpiderControl() { async_thread_->join(); }

void SpiderControl::spinThreadKHI() {
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(spider_control_node);
  executor.spin();
}

void SpiderControl::createSubscriber() {
  twist_sub =
      spider_control_node->create_subscription<geometry_msgs::msg::Twist>(
          "spider_driver/twist", 10,
          std::bind(&SpiderControl::twistCallback, this, _1));
}

void SpiderControl::createClient() {
  ik_calculator_client =
      spider_control_node->create_client<spider_msgs::srv::IK>(
          "spider/ik_calculator");
}

void SpiderControl::createInterfaceClient(std::vector<double> initial_pos) {
  spider_interface =
      std::make_shared<spider_client_library::SpiderClientImitation>(
          initial_pos);
}

std::vector<double> SpiderControl::getJointData() {
  std::vector<double> joints;
  spider_interface->getJointData(joints);
  return joints;
}

void SpiderControl::twistCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (!(ik_calculator_client->wait_for_service(1s))) {
    RCLCPP_ERROR(spider_control_node->get_logger(),
                 "/spider/ik_calculator not avaliable.");
    throw std::runtime_error("/spider/ik_calculator not avaliable");
  }

  auto request_ik = std::make_shared<spider_msgs::srv::IK::Request>();
  request_ik->offset.angular = msg->angular;
  request_ik->offset.linear = msg->linear;

  auto future_ik = ik_calculator_client->async_send_request(request_ik);

  if (!(rclcpp::spin_until_future_complete(spider_control_node, future_ik) ==
        rclcpp::FutureReturnCode::SUCCESS)) {
    RCLCPP_ERROR(spider_control_node->get_logger(),
                 "Failed to call service /spider/ik_calculator");
    throw std::runtime_error("Failed to call service /spider/ik_calculator");
  }

  RCLCPP_INFO(spider_control_node->get_logger(),
              "Success request in service spider/ik_calculator");
  auto ik_result = future_ik.get();
}

}  // namespace spider_driver
