/*
* Copyright (C) 2021 Alexander Junk <dev@junk.technology>
* 
* This program is free software: you can redistribute it and/or modify it 
* under the terms of the GNU Lesser General Public License as published 
* by the Free Software Foundation, either version 3 of the License, or 
* (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. 
* See the GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License 
* along with this program. If not, see <https://www.gnu.org/licenses/>. 
*/

#ifndef MECANUM_CONTROLLER_VELOCITY_HPP_INCLUDED
#define MECANUM_CONTROLLER_VELOCITY_HPP_INCLUDED

#include "controller_interface/controller_interface.hpp"

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "ros2_control_mecanum_controller/odometry.hpp"

#include <queue>

namespace mecanum_controller_velocity
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MecanumVelocityController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  MecanumVelocityController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };

  const char * feedback_type() const;
  CallbackReturn configure_wheel(const std::string & wheel_name,
    WheelHandle*& registered_handle);

  std::string front_left_wheel_name_;
  std::string front_right_wheel_name_;
  std::string back_left_wheel_name_;
  std::string back_right_wheel_name_;


  WheelHandle* registered_front_left_wheel_handle_{nullptr};
  WheelHandle* registered_front_right_wheel_handle_{nullptr};
  WheelHandle* registered_back_left_wheel_handle_{nullptr};
  WheelHandle* registered_back_right_wheel_handle_{nullptr};

  struct WheelParams
  {
    size_t wheels_per_side = 0;
    double separation_lr = 0.0;  // w.r.t. the midpoint of the wheel width
    double separation_fb = 0.0;  // w.r.t. the midpoint of the wheel width (front to back)
    double radius = 0.0;      // Assumed to be the same for both wheels
    double separation_multiplier = 1.0;
    double left_radius_multiplier = 1.0;
    double right_radius_multiplier = 1.0;
    double gearing = 1.0;
  } wheel_params_;

   struct OdometryParams
  {
    bool open_loop = false;
    bool position_feedback = false;
    bool enable_odom_tf = true;
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::array<double, 6> pose_covariance_diagonal{0};
    std::array<double, 6> twist_covariance_diagonal{0};
  } odom_params_;

  Odometry odometry_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;


  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{200};

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

  std::queue<Twist> previous_commands_;  // last two commands


  rclcpp::Time previous_update_timestamp_{0};

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0};

  bool is_halted = false;
  bool use_stamped_vel_ = false;

  bool reset();
  void halt();
};



}

#endif // MECANUM_CONTROLLER_VELOCITY_HPP_INCLUDED
