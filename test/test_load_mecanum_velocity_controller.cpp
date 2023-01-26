// Copyright 2020 Alexander Junk
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

#include <gmock/gmock.h>
#include <memory>
#include <string>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"


TEST(TestLoadMecanumVelocityController, load_controller)
{
   std::string hardware_system_odrive =
      R"(
      <ros2_control name="odrive_system" type="system">
        <hardware>
          <plugin>ros2_control_odrive_hw/ODriveSystemHardware</plugin>
          <param name="serial_number">000000</param>
        </hardware>

        <joint name="axis0">
          <command_interface name="velocity"/>
          <state_interface name="velocity"/>
        </joint>
        <joint name="axis1">
          <command_interface name="velocity"/>
          <state_interface name="velocity"/>
      </joint>
      </ros2_control>
      
    )";


  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(ros2_control_test_assets::urdf_head + hardware_system_odrive + ros2_control_test_assets::urdf_tail),
    executor, "test_controller_manager");

  ASSERT_NO_THROW(
    cm.load_controller("test_mecanum_velocity_controller", "ros2_control_mecanum_controller/MecanumVelocityController"));

  rclcpp::shutdown();
}