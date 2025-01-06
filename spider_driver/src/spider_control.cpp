#include <spider_control/spider_control.h>

namespace spider_driver {

SpiderControl::SpiderControl()
    : spider_control_node(
          std::make_shared<rclcpp::Node>("spider_control_node")) {
  async_thread_ =
      std::make_shared<std::thread>(&SpiderControl::spinThreadKHI, this);
}

SpiderControl::~SpiderControl() { async_thread_->join(); }

void SpiderControl::spinThreadKHI() {
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(spider_control_node);
  executor.spin();
}

}  // namespace spider_driver
