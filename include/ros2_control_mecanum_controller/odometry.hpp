// Copyright 2021 Alexander Junk <dev@junk.technology>
// Author: Alexander Junk 

// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 */

#ifndef MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include <cmath>

#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

namespace mecanum_controller_velocity
{
class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  void init(const rclcpp::Time & time);
  bool updateFromVelocity(double front_left_vel, double front_right_vel, double back_left_vel, double back_right_vel, const rclcpp::Time & time);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  std::tuple<double, double> getLinear() const { return {linear_x_, linear_y_}; }
  double getAngular() const { return angular_; }

  void setWheelParams(double wheel_separation_lr, double wheel_separation_fb, double wheel_radius, double gearing);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

  void resetAccumulators();

  // Current timestamp:
  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_x_;   //   [m/s]
  double linear_y_;   //   [m/s]
  double angular_;  // [rad/s]

  // Wheel kinematic parameters [m]:
  double wheel_separation_lr_{0.0};  // w.r.t. the midpoint of the wheel width
  double wheel_separation_fb_{0.0};  // w.r.t. the midpoint of the wheel width (front to back)
  double wheel_radius_       {0.0};  // Assumed to be the same for both wheels
  double gearing_            {0.0};

  // Previous wheel position/state [rad]:
  double left_wheel_old_pos_;
  double right_wheel_old_pos_;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_x_accumulator_;
  RollingMeanAccumulator linear_y_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace mecanum_controller_velocity

#endif  // MECANUM_DRIVE_CONTROLLER__ODOMETRY_HPP_