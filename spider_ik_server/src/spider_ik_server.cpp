#include <spider_ik_server/spider_ik_server.h>


namespace spider_ik {

IkServers::IkServers(rclcpp::NodeOptions options)
    : Node("spider_ik",
           options.allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true)) {
  RCLCPP_INFO_STREAM(this->get_logger(), "IkServers : READY");

  getRosParam();
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

// void IkServers::createService() {
//   service_ik = create_service<geometry_msgs::msg::Twist>(
//       "spider_twist", std::bind(&IkServers::getPosErrorCallback, this,
//                                 std::placeholders::_1,
//                                 std::placeholders::_2));
// }
}  // namespace spider_ik

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(spider_ik::IkServers)
