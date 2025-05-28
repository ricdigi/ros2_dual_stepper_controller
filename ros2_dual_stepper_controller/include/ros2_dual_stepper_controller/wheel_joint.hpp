
#ifndef WHEEL_JOINT_HPP
#define WHEEL_JOINT_HPP

#include <string>

struct WheelJoint {
  std::string name;

  double commanded_velocity_rad_s = 0.0;

  double position_rad = 0.0;               // current raw encoder reading
  double previous_position_rad = 0.0;      // previous position in radians

  double total_position_rad = 0.0;         // unwrapped continuous position
  double last_total_position_rad = 0.0;    // last total unwrapped position

  double velocity_rad_s = 0.0;             // current angular velocity in rad/s

  double updateTotalPositionRad();
  void computeVelocityRadS(double dt);

};

#endif //WHEEL_JOINT_H
