#include <spider_control/spider_hardware_interface.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
namespace spider_driver {

hardware_interface::CallbackReturn SpiderHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& hardware_info) {
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::vector<double> initial_pos;
  for (auto info_joint : info_.joints) {
    for (auto state_interface_joint : info_joint.state_interfaces) {
      if (state_interface_joint.name == "position") {
        initial_pos.push_back(std::stod(state_interface_joint.initial_value));
      }
    }
  }

  spider_data.resize(info_.joints.size() / 3);

  // for (int i = 0; i < initial_pos.size(); i++) {
  //   int index_leg = i / 3;

  //   spider_data[index_leg].coxa.position.state = initial_pos[index_leg * 3];
  //   spider_data[index_leg].femur.position.state =
  //       initial_pos[index_leg * 3 + 1];
  //   spider_data[index_leg].tibia.position.state =
  //       initial_pos[index_leg * 3 + 2];
  // }
  spider_control = std::make_shared<SpiderControl>(initial_pos);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SpiderHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t leg = 0; leg < names_leg.size(); leg++) {
    spider_data[leg].name = names_leg[leg];

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        names_coxa[leg], hardware_interface::HW_IF_POSITION,
        &spider_data[leg].coxa.position.state));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        names_femur[leg], hardware_interface::HW_IF_POSITION,
        &spider_data[leg].femur.position.state));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        names_tibia[leg], hardware_interface::HW_IF_POSITION,
        &spider_data[leg].tibia.position.state));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        names_coxa[leg], hardware_interface::HW_IF_VELOCITY,
        &spider_data[leg].coxa.velocity.state));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        names_femur[leg], hardware_interface::HW_IF_VELOCITY,
        &spider_data[leg].femur.velocity.state));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        names_tibia[leg], hardware_interface::HW_IF_VELOCITY,
        &spider_data[leg].tibia.velocity.state));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SpiderHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t leg = 0; leg < names_leg.size(); leg++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        names_coxa[leg], hardware_interface::HW_IF_POSITION,
        &spider_data[leg].coxa.position.command));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        names_femur[leg], hardware_interface::HW_IF_POSITION,
        &spider_data[leg].femur.position.command));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        names_tibia[leg], hardware_interface::HW_IF_POSITION,
        &spider_data[leg].tibia.position.command));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn SpiderHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpiderHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type SpiderHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  auto joints = spider_control->getJointData();

  for (int i = 0; i < joints.size(); i++) {
    int index_leg = i / 3;
    spider_data[index_leg].coxa.position.state = joints[index_leg * 3];
    spider_data[index_leg].femur.position.state = joints[index_leg * 3 + 1];
    spider_data[index_leg].tibia.position.state = joints[index_leg * 3 + 2];
  }

  return hardware_interface::return_type::OK;
}
hardware_interface::return_type SpiderHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SpiderHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SpiderHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {
  // switch between position and velocity modes
  return hardware_interface::return_type::OK;
}

// struct
InterfaceInfo::InterfaceInfo() : command(0.0), state(0.0) {}

InterfaceType::InterfaceType()
    : position(InterfaceInfo()),
      velocity(InterfaceInfo()),
      effort(InterfaceInfo()) {}

SpiderLeg::SpiderLeg()
    : coxa(InterfaceType()), femur(InterfaceType()), tibia(InterfaceType()) {}

}  // namespace spider_driver
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(spider_driver::SpiderHardwareInterface,
                       hardware_interface::SystemInterface)