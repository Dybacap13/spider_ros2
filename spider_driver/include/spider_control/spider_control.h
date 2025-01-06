
#include <rclcpp/rclcpp.hpp>
namespace spider_driver {

class SpiderControl {
 public:
  SpiderControl();
  ~SpiderControl();

 private:
  void spinThreadKHI();
  void getRosParam();
  void createService();

  std::shared_ptr<std::thread> async_thread_;
  std::shared_ptr<rclcpp::Node> spider_control_node;
};

}  // namespace spider_driver