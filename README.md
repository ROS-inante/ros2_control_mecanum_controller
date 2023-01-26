# ros2_control_mecanum_controller

ROS2 controller for mecanum wheel kinematics.

## Overview
This repository provides a controller for mecanum wheel kinematics for the [ros2_control framework](https://control.ros.org/master/index.html).

Platform specific parameters can be set via ROS2 parameters.

Only velocity feedback is supported at the moment.

## Parameters

| Parameter |  Type  | Description |
|:-----|:--------:|:---|
| `front_left_wheel_name` | string | Joint name for front left wheel as used by ros2_control hardware component.|
| `front_right_wheel_name` | string  | Joint name for front right wheel as used by ros2_control hardware component. |
| `back_left_wheel_name` | string | Joint name for back left wheel as used by ros2_control hardware component. |
| `back_right_wheel_name` | string | Joint name for back left wheel as used by ros2_control hardware component. |
| `wheel_separation_lr` | meters | Distance between left and right wheels (wheel center to wheel center). |
| `wheel_separation_fb` | | Distance between front and back wheels (wheel center to wheel center). |
| `wheel_radius` | meters | Wheel radius.|
| `publish_rate` | Hz | Publishing rate for the odometry.|
| `cmd_vel_timeout` | ms | Length of time since last command message after which motion is halted. |
| `velocity_rolling_window_size` | integer | Window size used for the RollingMeanAccumulator used for odometry. |
| `use_stamped_vel` | bool | Specify wether to use timestamped or non-timestamped velocity messages.|
| `odom_frame_id` | string | ID string for the odometry reference frame (for TF). |
| `base_frame_id` | string | ID string for the robot base reference frame (for TF). |
| `pose_covariance_diagonal` | double | Diagonal of the pose covariance matrix which gets attached to the generated odometry messages.|
| `twist_covariance_diagonal` | double  | Diagonal of the twist covariance matrix which gets attached to the generated odometry messages. |




## Attributions
The odometry code was adapted from the ROS2 differential drive examples found in the [ros-controls repository][def].



[def]: https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller