
#ifndef WHEEL_JOINT_HPP
#define WHEEL_JOINT_HPP

#include <string>

struct WheelJoint {
  std::string name;

  float commanded_velocity_rad_s = 0.0;

  float position_rad = 0.0;               // current raw encoder reading
  float previous_position_rad = 0.0;      // previous position in radians

  float total_position_rad = 0.0;         // unwrapped continuous position
  float last_total_position_rad = 0.0;    // last total unwrapped position

  float velocity_rad_s = 0.0;             // current angular velocity in rad/s

  float updateTotalPositionRad();
  void computeVelocityRadS(float dt);

};

#endif //WHEEL_JOINT_H
