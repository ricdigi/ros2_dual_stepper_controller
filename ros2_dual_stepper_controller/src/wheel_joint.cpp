
#include "ros2_dual_stepper_controller/wheel_joint.hpp"
#include <cmath>


// Update the total position and manage wrap around
double WheelJoint::updateTotalPositionRad() {
    double delta = position_rad - previous_position_rad;

    if (std::abs(delta) > M_PI) {
        if (delta > 0) delta -= 2 * M_PI;
        else delta += 2 * M_PI;
    }

    total_position_rad += delta;
    previous_position_rad = position_rad;

    return delta;
}


// Compute the angular velocity in rad/s
void WheelJoint::computeVelocityRadS(double dt, const rclcpp::Logger &logger) {
    double delta = total_position_rad - last_total_position_rad;
    double new_velocity = delta / dt;
    velocity_rad_s = new_velocity;  // or apply smoothing

    RCLCPP_INFO(logger, "Δθ=%.4f rad, dt=%.4f s, ω=%.4f rad/s", delta, dt, velocity_rad_s);

    last_total_position_rad = total_position_rad;
}



