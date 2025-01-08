
#include <rclcpp/rclcpp.hpp>
#include <spider_msgs/srv/ik.hpp>

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

  // parametrs
  std::vector<double> COXA_TO_CENTER_X, COXA_TO_CENTER_Y;
  std::vector<double> INIT_COXA_ANGLE;
  // std::vector<double> INIT_FOOT_POS_X, INIT_FOOT_POS_Y, INIT_FOOT_POS_Z;
  double COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH;
  int NUMBER_OF_LEGS;
  template <typename T>
  void readRosParam(const std::string param_name, T &param_value);

  // servers
  //   rclcpp::Service<geometry_msgs::msg::Twist>::SharedPtr service_ik;
  //   void getCalculateIk(
  //       const std::shared_ptr<mts_calibration::srv::Calibration::Request>
  //       request, std::shared_ptr<mts_calibration::srv::Calibration::Response>
  //       response) {}
};

}  // namespace spider_ik