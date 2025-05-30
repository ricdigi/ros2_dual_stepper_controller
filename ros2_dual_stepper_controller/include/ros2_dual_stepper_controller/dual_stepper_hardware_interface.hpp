#ifndef DUAL_STEPPER_HARDWARE_INTERFACE_HPP
#define DUAL_STEPPER_HARDWARE_INTERFACE_HPP

#include "ros2_dual_stepper_controller/serial_comm.hpp"
#include "ros2_dual_stepper_controller/wheel_joint.hpp"

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dual_stepper_hardware_interface {
class DualStepperHardwareInterface : public hardware_interface::SystemInterface {

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DualStepperHardwareInterface)

   /*
    * In the on_init() callback i should initialize all memebr variables and process the parameters from the info
    * argument which come directly from parsing the URDF file. In the first line usually the parents on_init is called
    * to process standard values, like name.
    */
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    /*
    * In the on_configure() callback i should setup the communication to the hardware and set everything up
    * so that the hardware can be activated.
    */
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    void readEncoderData(const rclcpp::Duration & period);
    bool sendVelocityCommand();

    // Get the logger of the SystemInterface.
    rclcpp::Logger get_logger() const { return *logger_; }

    // Get the clock of the SystemInterface.
    rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

  private:
    SerialComm serial_comm_;

    WheelJoint left_wheel_;
    WheelJoint right_wheel_;

    // Objects for logging
    std::shared_ptr<rclcpp::Logger> logger_;
    rclcpp::Clock::SharedPtr clock_;

};

} // namespace dual_stepper_hardware_interface

#endif //DUAL_STEPPER_HARDWARE_INTERFACE_H
