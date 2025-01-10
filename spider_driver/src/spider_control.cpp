
#include <spider_control/spider_control.h>
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std::chrono_literals;
using namespace std::placeholders;
namespace spider_driver {

SpiderControl::SpiderControl(std::vector<double> initial_pose)
    : spider_control_node(
          std::make_shared<rclcpp::Node>("spider_control_node")) {
  createInterfaceClient(initial_pose);
  createAction();
  async_thread_ =
      std::make_shared<std::thread>(&SpiderControl::spinThreadKHI, this);
}

SpiderControl::~SpiderControl() { async_thread_->join(); }

void SpiderControl::spinThreadKHI() {
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(spider_control_node);
  executor.spin();
}
void SpiderControl::createAction() {
  RCLCPP_INFO_STREAM(spider_control_node->get_logger(),
                     "name : " << action_name_trajectory);
  action_move = rclcpp_action::create_server<JointTrajectoryMsg>(
      spider_control_node, action_name_trajectory,
      std::bind(&SpiderControl::handleGoalTrajectory, this, _1, _2),
      std::bind(&SpiderControl::handleCancelTrajectory, this, _1),
      std::bind(&SpiderControl::handleAcceptedTrajectory, this, _1));
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

// void SpiderControl::twistCallback(
//     const geometry_msgs::msg::Twist::SharedPtr msg) {
//   std::cout << "Start" << std::endl;
//   // try {
//   if (!(ik_calculator_client->wait_for_service(1s))) {
//     RCLCPP_ERROR(spider_control_node->get_logger(),
//                  "/spider/ik_calculator not avaliable.");
//     throw std::runtime_error("/spider/ik_calculator not avaliable");
//   }

//   auto request_ik = std::make_shared<spider_msgs::srv::IK::Request>();
//   request_ik->offset.angular = msg->angular;
//   request_ik->offset.linear = msg->linear;
//   request_ik->joints = getJointData();

//   auto future_ik = ik_calculator_client->async_send_request(request_ik);

//   }

//   RCLCPP_INFO(spider_control_node->get_logger(),
//               "Success request in service spider/ik_calculator");
//   auto ik_result = future_ik.get();
// }

/************************************************************************************/
// action
rclcpp_action::GoalResponse SpiderControl::handleGoalTrajectory(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const JointTrajectoryMsg::Goal> /*goal*/) {
  RCLCPP_INFO_STREAM(spider_control_node->get_logger(),
                     "Send trajectory to robot.");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SpiderControl::handleCancelTrajectory(
    const std::shared_ptr<JointTrajectoryGoal> goal_handle) {
  RCLCPP_INFO_STREAM(spider_control_node->get_logger(),
                     "Received request to cancel goal");
  (void)goal_handle;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void SpiderControl::handleAcceptedTrajectory(
    const std::shared_ptr<JointTrajectoryGoal> goal_handle) {
  RCLCPP_INFO_STREAM(spider_control_node->get_logger(),
                     "Accepted handleAcceptedTrajectory");
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&SpiderControl::executeTrajectory, this, _1),
              goal_handle}
      .detach();
}

void SpiderControl::executeTrajectory(
    const std::shared_ptr<JointTrajectoryGoal> goal_handle) {
  auto result = std::make_shared<JointTrajectoryMsg::Result>();
  try {
    RCLCPP_INFO_STREAM(spider_control_node->get_logger(), "Execute trajectory");

    goal_handle->succeed(result);

  } catch (const std::exception& e) {
    goal_handle->abort(result);
    RCLCPP_ERROR_STREAM(spider_control_node->get_logger(),
                        "Failed moveByTrajectory: " << e.what());
  }

  RCLCPP_INFO_STREAM(spider_control_node->get_logger(),
                     std::endl
                         << std::endl
                         << std::endl
                         << std::endl
                         << "Execute trajectory complete" << std::endl
                         << std::endl
                         << std::endl
                         << std::endl
                         << std::endl);
}

/************************************************************************************/

// void SpiderControl::executeTrajectory() {
//   std::cout << "executeTrajectory!!!" << std::endl;
//   // const std::lock_guard<std::mutex> lock(mu);
//   std::cout << "trajectory.size() =  " << trajectory.size() << std::endl;
//   for (size_t index = 0; index < trajectory.size(); index++) {
//     spider_interface->writeJointCommandPosition(trajectory[index].joints);
//     RCLCPP_INFO(spider_control_node->get_logger(), "1 1 1");
//     std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     RCLCPP_INFO(spider_control_node->get_logger(), "2 2 2");
//   }
//   RCLCPP_INFO(spider_control_node->get_logger(), "Execute trajectory
//   success");
// }
}  // namespace spider_driver
