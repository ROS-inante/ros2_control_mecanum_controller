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
*
*/

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

#include "ros2_control_mecanum_controller/mecanum_controller_velocity.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel_stamped";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace


namespace mecanum_controller_velocity
{

using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

MecanumVelocityController::MecanumVelocityController() : controller_interface::ControllerInterface() {
    
}

const char * MecanumVelocityController::feedback_type() const
{
    // ToDo: Make feedback type configurable again.
    return HW_IF_VELOCITY;
  //return odom_params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

CallbackReturn MecanumVelocityController::on_init()
{
  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    auto_declare<std::string>("front_left_wheel_name", std::string());
    auto_declare<std::string>("front_right_wheel_name", std::string());

    auto_declare<std::string>("back_left_wheel_name", std::string());
    auto_declare<std::string>("back_right_wheel_name", std::string());

    auto_declare<double>("wheel_separation_lr", wheel_params_.separation_lr);
    auto_declare<double>("wheel_separation_fb", wheel_params_.separation_fb);
    auto_declare<double>("wheel_radius", wheel_params_.radius);
    auto_declare<double>("wheel_separation_multiplier", wheel_params_.separation_multiplier);
    auto_declare<double>("left_wheel_radius_multiplier", wheel_params_.left_radius_multiplier);
    auto_declare<double>("right_wheel_radius_multiplier", wheel_params_.right_radius_multiplier);

    auto_declare<double>("publish_rate", 10);

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    auto_declare<int>("velocity_rolling_window_size", 10);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

    auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
    auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);
    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());


  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration MecanumVelocityController::command_interface_configuration() const
{
    std::vector<std::string> conf_names;
    
    conf_names.push_back(front_left_wheel_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(front_right_wheel_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(back_left_wheel_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(back_right_wheel_name_ + "/" + HW_IF_VELOCITY);
    
    return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration MecanumVelocityController::state_interface_configuration() const
{
    std::vector<std::string> conf_names;

    conf_names.push_back(front_left_wheel_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(front_right_wheel_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(back_left_wheel_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(back_right_wheel_name_ + "/" + HW_IF_VELOCITY);

    return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type MecanumVelocityController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
    auto logger = node_->get_logger();
    const auto current_time = time;

    // RCLCPP_INFO(logger, "Updating");

    if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
    {
        if (!is_halted)
        {
        halt();
        is_halted = true;
        }
        return controller_interface::return_type::OK;
    }

    std::shared_ptr<Twist> last_command_msg;
    received_velocity_msg_ptr_.get(last_command_msg);

    if (last_command_msg == nullptr)
    {
        RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
        return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = current_time - last_command_msg->header.stamp;
    // Brake if cmd_vel has timeout, override the stored command
    if (age_of_last_command > cmd_vel_timeout_)
    {
        last_command_msg->twist.linear.x = 0.0;
        last_command_msg->twist.linear.y = 0.0;
        last_command_msg->twist.angular.z = 0.0;
    }

    // command may be limited further by SpeedLimit,
    // without affecting the stored twist command
    Twist command = *last_command_msg;
    auto v_x = command.twist.linear.x;
    auto v_y = command.twist.linear.y;
    auto w_z = command.twist.angular.z;


    std::array<double, 4> w;
    w[0] = registered_front_left_wheel_handle_->feedback.get().get_value()/wheel_params_.gearing;
    w[1] = registered_front_right_wheel_handle_->feedback.get().get_value()/wheel_params_.gearing;
    w[2] = registered_back_left_wheel_handle_->feedback.get().get_value()/wheel_params_.gearing;
    w[3] = registered_back_right_wheel_handle_->feedback.get().get_value()/wheel_params_.gearing;

    if (std::isnan(w[0]) || std::isnan(w[1]) || std::isnan(w[2]) || std::isnan(w[3]))
    {
        RCLCPP_ERROR(
            logger, "One of the wheel values is invalid for feedback type %s.", feedback_type());
        return controller_interface::return_type::ERROR;
    }
    odometry_.updateFromVelocity(w[0], w[1], w[2], w[3], current_time);

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, odometry_.getHeading());

    // if (previous_publish_timestamp_ + publish_period_ < current_time)
    // {
    //     previous_publish_timestamp_ += publish_period_;

    //     if (realtime_odometry_publisher_->trylock())
    //     {
    //         auto & odometry_message = realtime_odometry_publisher_->msg_;
    //         odometry_message.header.stamp = current_time;

    //         auto [ o_x, o_y ] = odometry_.getLinear();
    //         odometry_message.twist.twist.linear.x = o_x;
    //         odometry_message.twist.twist.linear.y = o_y;
    //         odometry_message.twist.twist.angular.z = odometry_.getAngular();

    //         realtime_odometry_publisher_->unlockAndPublish();
    //     }

    // }

     if (realtime_odometry_publisher_->trylock())
        {
            auto & odometry_message = realtime_odometry_publisher_->msg_;
            odometry_message.header.stamp = current_time;

            auto [ o_x, o_y ] = odometry_.getLinear();
            odometry_message.twist.twist.linear.x = o_x;
            odometry_message.twist.twist.linear.y = o_y;
            odometry_message.twist.twist.angular.z = odometry_.getAngular();

            realtime_odometry_publisher_->unlockAndPublish();
    }

    // Calculate intermediate values and wheel speeds
    auto l_x = wheel_params_.separation_lr;
    auto l_y = wheel_params_.separation_fb;
    auto r   = wheel_params_.radius;

    w[0] = 1/r * (v_x - v_y - (l_x + l_y)*w_z) / (2 * M_PI) * wheel_params_.gearing;
    w[1] = 1/r * (v_x + v_y + (l_x + l_y)*w_z) / (2 * M_PI) * wheel_params_.gearing;
    w[2] = 1/r * (v_x + v_y - (l_x + l_y)*w_z) / (2 * M_PI) * wheel_params_.gearing;
    w[3] = 1/r * (v_x - v_y + (l_x + l_y)*w_z) / (2 * M_PI) * wheel_params_.gearing;

    registered_front_left_wheel_handle_->velocity.get().set_value(w[0]);
    registered_front_right_wheel_handle_->velocity.get().set_value(w[1]);
    registered_back_left_wheel_handle_->velocity.get().set_value(w[2]);
    registered_back_right_wheel_handle_->velocity.get().set_value(w[3]);

    // RCLCPP_INFO(logger, "%lf | %lf | %lf | %lf.", w[0], w[1], w[2], w[3]);
  


    return controller_interface::return_type::OK;
}

CallbackReturn MecanumVelocityController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = node_->get_logger();

    RCLCPP_INFO(logger, "CONFIGURING.");
    
    try{
        
        // update parameters
        front_left_wheel_name_ = node_->get_parameter("front_left_wheel_name").as_string();
        front_right_wheel_name_ = node_->get_parameter("front_right_wheel_name").as_string();
        back_left_wheel_name_ = node_->get_parameter("back_left_wheel_name").as_string();
        back_right_wheel_name_ = node_->get_parameter("back_right_wheel_name").as_string();

        if (front_right_wheel_name_.empty() || front_left_wheel_name_.empty() || back_right_wheel_name_.empty() || back_left_wheel_name_.empty())
        {
            RCLCPP_ERROR(
            logger, "Not all wheel joints specified. Got [%s], [%s], [%s], [%s],",
                front_right_wheel_name_.c_str(), front_left_wheel_name_.c_str(), back_right_wheel_name_.c_str(), back_left_wheel_name_.c_str());
            return CallbackReturn::ERROR;
        }


        wheel_params_.separation_lr = node_->get_parameter("wheel_separation_lr").as_double();
        wheel_params_.separation_fb = node_->get_parameter("wheel_separation_fb").as_double();
        wheel_params_.radius = node_->get_parameter("wheel_radius").as_double();
        wheel_params_.separation_multiplier =
            node_->get_parameter("wheel_separation_multiplier").as_double();
        wheel_params_.left_radius_multiplier =
            node_->get_parameter("left_wheel_radius_multiplier").as_double();
        wheel_params_.right_radius_multiplier =
            node_->get_parameter("right_wheel_radius_multiplier").as_double();
        wheel_params_.gearing =
            node_->get_parameter("gearing").as_double();

        odometry_.setWheelParams(wheel_params_.separation_lr, wheel_params_.separation_fb,  wheel_params_.radius, wheel_params_.gearing);
        odometry_.setVelocityRollingWindowSize(
            node_->get_parameter("velocity_rolling_window_size").as_int());

        odom_params_.odom_frame_id = node_->get_parameter("odom_frame_id").as_string();
        odom_params_.base_frame_id = node_->get_parameter("base_frame_id").as_string();

        auto pose_diagonal = node_->get_parameter("pose_covariance_diagonal").as_double_array();
        std::copy(
            pose_diagonal.begin(), pose_diagonal.end(), odom_params_.pose_covariance_diagonal.begin());

        auto twist_diagonal = node_->get_parameter("twist_covariance_diagonal").as_double_array();
        std::copy(
            twist_diagonal.begin(), twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());


        cmd_vel_timeout_ = std::chrono::milliseconds{
            static_cast<int>(node_->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
        use_stamped_vel_ = node_->get_parameter("use_stamped_vel").as_bool();


        if (!reset())
        {
            RCLCPP_ERROR(node_->get_logger(), "ERROR RESETTING");
            return CallbackReturn::ERROR;
        }

        // limit the publication on the topics /odom and /tf
        publish_rate_ = node_->get_parameter("publish_rate").as_double();
        publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
        previous_publish_timestamp_ = node_->get_clock()->now();

    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(logger, "Configuring ERROR.");
        fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    // Configure TWIST subscriber

    const Twist empty_twist;
    received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

    // Fill last two commands with default constructed commands
    previous_commands_.emplace(empty_twist);
    previous_commands_.emplace(empty_twist);

    // initialize command subscriber
    if (use_stamped_vel_)
    {
        velocity_command_subscriber_ = node_->create_subscription<Twist>(
        DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<Twist> msg) -> void {
            if (!subscriber_is_active_)
            {
            RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
            }
            if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
            {
            RCLCPP_WARN_ONCE(
                node_->get_logger(),
                "Received TwistStamped with zero timestamp, setting it to current "
                "time, this message will only be shown once");
            msg->header.stamp = node_->get_clock()->now();
            }
            received_velocity_msg_ptr_.set(std::move(msg));
        });
    }
    else
    {
        velocity_command_unstamped_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
            if (!subscriber_is_active_)
            {
                RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
                return;
            }

            // Write fake header in the stored stamped command
            std::shared_ptr<Twist> twist_stamped;
            received_velocity_msg_ptr_.get(twist_stamped);
            twist_stamped->twist = *msg;
            twist_stamped->header.stamp = node_->get_clock()->now();
        });
    }

     // initialize odometry publisher and messasge
    odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
        DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
        odometry_publisher_);

    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.frame_id = odom_params_.odom_frame_id;
    odometry_message.child_frame_id = odom_params_.base_frame_id;

    // limit the publication on the topics /odom and /tf
    publish_rate_ = node_->get_parameter("publish_rate").as_double();
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
    previous_publish_timestamp_ = node_->get_clock()->now();

    // initialize odom values zeros
    odometry_message.twist =
        geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

    constexpr size_t NUM_DIMENSIONS = 6;
    for (size_t index = 0; index < 6; ++index)
    {
        // 0, 7, 14, 21, 28, 35
        const size_t diagonal_index = NUM_DIMENSIONS * index + index;

        odometry_message.pose.covariance[diagonal_index] = 
            odom_params_.pose_covariance_diagonal[index];

        odometry_message.twist.covariance[diagonal_index] =
            odom_params_.twist_covariance_diagonal[index];
    }

    previous_update_timestamp_ = node_->get_clock()->now();

    RCLCPP_INFO(logger, "Configuring DONE.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumVelocityController::on_activate(const rclcpp_lifecycle::State &)
{
    auto logger = node_->get_logger();

    RCLCPP_INFO(logger, "ACTIVATING");
    
    try
    {
         

        const auto front_left_result =
            configure_wheel(front_left_wheel_name_, registered_front_left_wheel_handle_);
        const auto front_right_result =
            configure_wheel(front_right_wheel_name_, registered_front_right_wheel_handle_);
        const auto back_left_result =
            configure_wheel(back_left_wheel_name_, registered_back_left_wheel_handle_);
        const auto back_right_result =
            configure_wheel(back_right_wheel_name_, registered_back_right_wheel_handle_);
    
        if (front_left_result == CallbackReturn::ERROR || front_right_result == CallbackReturn::ERROR 
            || back_left_result == CallbackReturn::ERROR || back_right_result == CallbackReturn::ERROR)
        {
            return CallbackReturn::ERROR;
        }

    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(logger, "ACTIVATE ERROR.");
        fprintf(stderr, "Exception thrown during activate stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
       

    // ToDo: Add checking mechanism to see if handle could be aquired.

    // if (registered_front_left_wheel_handle_.empty() || registered_front_right_wheel_handle_.empty()
    //     || registered_back_right_wheel_handle_.empty() || registered_back_right_wheel_handle_.empty())
    // {
    //     RCLCPP_ERROR(
    //     node_->get_logger(), "Some wheel interfaces are non-existent.");
    //     return CallbackReturn::ERROR;
    // }

    is_halted = false;
    subscriber_is_active_ = true;

    RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");

    return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumVelocityController::on_deactivate(const rclcpp_lifecycle::State &)
{
    subscriber_is_active_ = false;

    return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumVelocityController::on_cleanup(const rclcpp_lifecycle::State &)
{
    if (!reset())
    {
        return CallbackReturn::ERROR;
    }

    received_velocity_msg_ptr_.set(std::make_shared<Twist>());
 
    return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumVelocityController::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(node_->get_logger(), "GOT ERROR");


    if (!reset())
    {
        return CallbackReturn::ERROR;
    }
    
    return CallbackReturn::SUCCESS;
}

bool MecanumVelocityController::reset()
{
    RCLCPP_INFO(node_->get_logger(), "RESETTING");

    odometry_.resetOdometry();

    try
    {

    // release the old queue
    std::queue<Twist> empty;
    std::swap(previous_commands_, empty);

    // ToDo: Release old handles
    delete registered_front_left_wheel_handle_;
    delete registered_front_right_wheel_handle_;
    delete registered_back_left_wheel_handle_;
    delete registered_back_right_wheel_handle_;

    subscriber_is_active_ = false;
    //velocity_command_subscriber_.reset();
    //velocity_command_unstamped_subscriber_.reset();

    received_velocity_msg_ptr_.set(nullptr);
    is_halted = false;
    
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during reset with message: %s \n", e.what());
        return false;
    }


    return true;
}

CallbackReturn MecanumVelocityController::on_shutdown(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

void MecanumVelocityController::halt()
{
    const auto halt_wheels = [](auto  wheel_handle) {
        wheel_handle->velocity.get().set_value(0.0);
    };

    halt_wheels(registered_front_left_wheel_handle_);
    halt_wheels(registered_front_right_wheel_handle_);
    halt_wheels(registered_back_left_wheel_handle_);
    halt_wheels(registered_back_right_wheel_handle_);

}

CallbackReturn MecanumVelocityController::configure_wheel(const std::string& wheel_name,
    WheelHandle*& registered_handle)
{
    auto logger = node_->get_logger();

    try
    {

        if (wheel_name.empty())
        {
            RCLCPP_ERROR(logger, "No wheel name specified");
            return CallbackReturn::ERROR;
        }

        
        const auto interface_name = feedback_type();
        const auto state_handle = std::find_if(
            state_interfaces_.cbegin(), state_interfaces_.cend(),
            [&wheel_name, &interface_name](const auto & interface) {
                return interface.get_name() == wheel_name &&
                interface.get_interface_name() == interface_name;
        });

        if (state_handle == state_interfaces_.cend())
        {
            RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
            return CallbackReturn::ERROR;
        }

        const auto command_handle = std::find_if(
            command_interfaces_.begin(), command_interfaces_.end(),
                [&wheel_name](const auto & interface) {
                    return interface.get_name() == wheel_name &&
                        interface.get_interface_name() == HW_IF_VELOCITY;
        });

        if (command_handle == command_interfaces_.end())
        {
        RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
        return CallbackReturn::ERROR;
        }

        registered_handle =  new WheelHandle{std::ref(*state_handle), std::ref(*command_handle)};

    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during configuring wheel with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    

    return CallbackReturn::SUCCESS;

}

}  // namespace mecanum_controller_velocity

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  mecanum_controller_velocity::MecanumVelocityController, controller_interface::ControllerInterface)
