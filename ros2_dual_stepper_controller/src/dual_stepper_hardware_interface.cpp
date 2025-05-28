
#include "ros2_dual_stepper_controller/dual_stepper_hardware_interface.hpp"
#include "ros2_dual_stepper_controller/serial_comm.hpp"
#include "ros2_dual_stepper_controller/wheel_joint.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace dual_stepper_hardware_interface
{

hardware_interface::CallbackReturn DualStepperHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {

    // Call the parent class default on_init() method
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    /* The default on_init() method processes the HardwareInfo object, which contains hardware details parsed from the
     * robot's URDF. This parsing is performed by the Resource Manager using the function
     * parse_control_resources_from_urdf(), defined in
     * ros2_control/hardware_interface/include/hardware_interface/component_parser.hpp.
     */

    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("dual_stepper_hardware_interface"));
    clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

    if (info_.joints.size() != 2) {
        RCLCPP_FATAL(logger_, "Expected exactly 2 joints, got %zu", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Successfully initialized!");
    return hardware_interface::CallbackReturn::SUCCESS;

}


hardware_interface::CallbackReturn DualStepperHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {

    // Configure and initialize the serial port and baud rate from the hardware parameters
    serial_comm_ = SerialComm();
    serial_comm_.setSerialPort(info_.hardware_parameters.at("serial_port"));
    serial_comm_.setBaudRate(std::stoi(info_.hardware_parameters.at("baud_rate")));
    if (serial_comm_.init() != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize the wheel joints with the parameters from the hardware info
    left_wheel_.name = info_.joints[0].name;
    right_wheel_.name = info_.joints[1].name;

    RCLCPP_INFO(logger_, "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;

}


std::vector<hardware_interface::StateInterface> DualStepperHardwareInterface::export_state_interfaces() {
  // We need to set up a position and a velocity interface for each wheel
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.total_position_rad));
  state_interfaces.emplace_back(hardware_interface::StateInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.velocity_rad_s));

  state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.total_position_rad));
  state_interfaces.emplace_back(hardware_interface::StateInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.velocity_rad_s));

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> DualStepperHardwareInterface::export_command_interfaces() {
  // We need to set up a velocity command interface for each wheel
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.commanded_velocity_rad_s));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.commanded_velocity_rad_s));

  return command_interfaces;
}


hardware_interface::return_type DualStepperHardwareInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period) {

    readEncoderData(period);

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type DualStepperHardwareInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period) {

    if (!sendVelocityCommand()) {
        RCLCPP_ERROR(logger_, "Failed to send velocity command to the stepper motors.");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}


void DualStepperHardwareInterface::readEncoderData(const rclcpp::Duration & period) {
    uint8_t cmd;
    std::vector<uint8_t> data;

    if (serial_comm_.receive(cmd, data)) {
        if (cmd == SerialComm::ENC_CMD && data.size() == SerialComm::ENC_DATA_LEN) {
           std::memcpy(&left_wheel_.position_rad, &data[0], 4);
           std::memcpy(&right_wheel_.position_rad, &data[4], 4);

           left_wheel_.updateTotalPositionRad();
           right_wheel_.updateTotalPositionRad();

           left_wheel_.computeVelocityRadS(period.seconds());
           right_wheel_.computeVelocityRadS(period.seconds());

        }
    }
}


bool DualStepperHardwareInterface::sendVelocityCommand() {
    std::vector<uint8_t> data(SerialComm::VEL_DATA_LEN);
    uint8_t cmd = SerialComm::VEL_CMD;

    float left_speed = left_wheel_.commanded_velocity_rad_s;
    float right_speed = right_wheel_.commanded_velocity_rad_s;

    std::memcpy(&data[0], &left_speed, sizeof(float));
    std::memcpy(&data[4], &right_speed, sizeof(float));

    // Send the command via serial communication
    return serial_comm_.send(cmd, data);
}


} // namespace dual_stepper_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dual_stepper_hardware_interface::dual_stepper_hardware_interface, hardware_interface::SystemInterface)
