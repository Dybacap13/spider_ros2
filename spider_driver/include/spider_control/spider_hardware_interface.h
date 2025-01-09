// System
#include <limits>
#include <memory>
#include <string>
#include <vector>

// ros2_control hardware_interface

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include "hardware_interface/handle.hpp"

// ROS
#include <spider_control/spider_control.h>

#include <rclcpp/rclcpp.hpp>

namespace spider_driver {

struct InterfaceInfo {
  InterfaceInfo();
  double state;
  double command;
};

struct InterfaceType {
  InterfaceType();
  InterfaceInfo position;
  InterfaceInfo velocity;
  InterfaceInfo effort;
};

struct SpiderLeg {
  SpiderLeg();
  std::string name;
  InterfaceType coxa;
  InterfaceType femur;
  InterfaceType tibia;
};

std::vector<std::string> names_coxa = {"joint_coxa_rr", "joint_coxa_rm",
                                       "joint_coxa_rf", "joint_coxa_lr",
                                       "joint_coxa_lm", "joint_coxa_lf"};

std::vector<std::string> names_femur = {"joint_femur_rr", "joint_femur_rm",
                                        "joint_femur_rf", "joint_femur_lr",
                                        "joint_femur_lm", "joint_femur_lf"};

std::vector<std::string> names_tibia = {"joint_tibia_rr", "joint_tibia_rm",
                                        "joint_tibia_rf", "joint_tibia_lr",
                                        "joint_tibia_lm", "joint_tibia_lf"};

std::vector<std::string> names_leg = {"rr", "rm", "rf", "lr", "lm", "lf"};

class SpiderHardwareInterface : public hardware_interface::SystemInterface {
 public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& /*previous_state*/) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& /*previous_state*/) override;

  hardware_interface::return_type read(const rclcpp::Time& /*time*/,
                                       const rclcpp::Duration& /*period*/);
  hardware_interface::return_type write(const rclcpp::Time& /*time*/,
                                        const rclcpp::Duration& /*period*/);

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& /*start_interfaces*/,
      const std::vector<std::string>& /*stop_interfaces*/) override;

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& /*start_interfaces*/,
      const std::vector<std::string>& /*stop_interfaces*/) override;

 private:
  std::vector<SpiderLeg> spider_data;
  std::shared_ptr<spider_driver::SpiderControl> spider_control;
};

}  // namespace spider_driver