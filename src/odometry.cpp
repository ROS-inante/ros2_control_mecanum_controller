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
 * Author: Enrique Fernández
 */

#include "ros2_control_mecanum_controller/odometry.hpp"
#include <cmath>

namespace mecanum_controller_velocity
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x_(0.0),
  linear_y_(0.0),
  angular_(0.0),
  wheel_separation_lr_(0.3),
  wheel_separation_fb_(0.28),
  wheel_radius_(0.048),
  left_wheel_old_pos_(0.0),
  right_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_x_accumulator_(velocity_rolling_window_size),
  linear_y_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::updateFromVelocity(double front_left_vel, double front_right_vel, double back_left_vel, double back_right_vel, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();

  std::array<double, 4> w;
  w[0] = 2*M_PI * front_left_vel;
  w[1] = 2*M_PI * front_right_vel;
  w[2] = 2*M_PI * back_left_vel;
  w[3] = 2*M_PI * back_right_vel;


  // Compute linear and angular diff:
  const double linear_x = wheel_radius_/4 * ( w[0] + w[1] + w[2] + w[3]);
  const double linear_y = wheel_radius_/4 * (-w[0] + w[1] + w[2] - w[3]);

  const double l_xy = wheel_separation_lr_ + wheel_separation_fb_;
  const double angular  = wheel_radius_/4 * (-w[0] + w[1] - w[2] + w[3]) / l_xy;

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  // linear_x_accumulator_.accumulate(linear_x);
  // linear_y_accumulator_.accumulate(linear_y);
  // angular_accumulator_.accumulate(angular);

  //linear_x_ = linear_x_accumulator_.getRollingMean();
  //linear_y_ = linear_y_accumulator_.getRollingMean();
  //angular_  = angular_accumulator_.getRollingMean();

  linear_x_ = linear_x;
  linear_y_ = linear_y;
  angular_  = angular;

  return true;
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(double wheel_separation_lr, double wheel_separation_fb, double wheel_radius, double gearing)
{
  wheel_separation_lr_ = wheel_separation_lr;
  wheel_separation_fb_ = wheel_separation_fb;
  wheel_radius_        = wheel_radius;
  gearing_ = gearing;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

// void Odometry::integrateRungeKutta2(double linear, double angular)
// {
//   const double direction = heading_ + angular * 0.5;

//   /// Runge-Kutta 2nd order integration:
//   x_ += linear * cos(direction);
//   y_ += linear * sin(direction);
//   heading_ += angular;
// }

// void Odometry::integrateExact(double linear_x, double linear_y, double angular)
// {
//   if (fabs(angular) < 1e-6)
//   {
//     integrateRungeKutta2(linear, angular);
//   }
//   else
//   {
//     /// Exact integration (should solve problems when angular is zero):
//     const double heading_old = heading_;
//     const double r = linear / angular;
//     heading_ += angular;
//     x_ += r * (sin(heading_) - sin(heading_old));
//     y_ += -r * (cos(heading_) - cos(heading_old));
//   }
// }

void Odometry::resetAccumulators()
{
  linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace mecanum_controller_velocity
