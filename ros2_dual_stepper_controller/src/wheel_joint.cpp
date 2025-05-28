
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
void WheelJoint::computeVelocityRadS(double dt) {
    double delta = total_position_rad - last_total_position_rad;
    velocity_rad_s = delta / dt;
    last_total_position_rad = total_position_rad;
}



