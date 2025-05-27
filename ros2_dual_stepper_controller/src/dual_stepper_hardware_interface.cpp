
#include "ros2_dual_stepper_controller/dual_stepper_hardware_interface.hpp"
#include "ros2_dual_stepper_controller/serial_comm.h"
#include "ros2_dual_stepper_controller/wheel_joint.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace dual_stepper_hardware_interface
{

hardware_interface::CallbackReturn dual_stepper_hardware_interface::on_init(const hardware_interface::HardwareInfo & info) {

    // Call the parent class default on_init() method
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    /* The default on_init() method processes the HardwareInfo object, which contains hardware details parsed from the
     * robot's URDF. This parsing is performed by the Resource Manager using the function
     * parse_control_resources_from_urdf(), defined in
     * ros2_control/hardware_interface/include/hardware_interface/component_parser.hpp.
     */

    if (info_.joints.size() != 2) {
        RCLCPP_FATAL(get_logger(), "Expected exactly 2 joints, got %zu", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Successfully initialized!");
    return hardware_interface::CallbackReturn::SUCCESS;

}


hardware_interface::CallbackReturn dual_stepper_hardware_interface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {

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


    for (const auto & [name, descr] : joint_state_interfaces_) { set_state(name, 0.0); }
    for (const auto & [name, descr] : joint_command_interfaces_) { set_command(name, 0.0); }

    RCLCPP_INFO(get_logger(), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;

}


hardware_interface::return_type dual_stepper_hardware_interface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period) {

    readEncoderData(period);

    set_state("left_wheel/position", left_wheel_.total_position_rad);
    set_state("left_wheel/velocity", left_wheel_.velocity_rad_s);

    set_state("right_wheel/position", right_wheel_.total_position_rad);
    set_state("right_wheel/velocity", right_wheel_.velocity_rad_s);

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type dual_stepper_hardware_interface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period) {

    left_wheel_.commanded_velocity_rad_s = get_command("left_wheel/velocity");
    right_wheel_.commanded_velocity_rad_s = get_command("right_wheel/velocity");

    if (!sendVelocityCommand()) {
        RCLCPP_ERROR(get_logger(), "Failed to send velocity command to the stepper motors.");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}


void dual_stepper_hardware_interface::readEncoderData(const rclcpp::Duration & period) {
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


bool dual_stepper_hardware_interface::sendVelocityCommand() {
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