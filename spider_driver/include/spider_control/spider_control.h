
#include <spider_client_imitation.h>
#include <spider_client_interface.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spider_msgs/srv/ik.hpp>
namespace spider_driver {

class SpiderControl {
 public:
  SpiderControl(std::vector<double> initial_pose);
  ~SpiderControl();
  std::vector<double> getJointData();

 private:
  void spinThreadKHI();
  void getRosParam();
  void createService();
  void createSubscriber();
  void createClient();
  void createInterfaceClient(std::vector<double> initial_pose);

  std::shared_ptr<std::thread> async_thread_;
  std::shared_ptr<rclcpp::Node> spider_control_node;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Client<spider_msgs::srv::IK>::SharedPtr ik_calculator_client;

  std::shared_ptr<spider_client_library::SpiderClientInterface>
      spider_interface;
};

}  // namespace spider_driver