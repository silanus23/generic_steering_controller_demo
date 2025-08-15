# Generic Steering Controller Demo

This repository is a **basic proof of concept** for integrating a generic steering controller with ROS 2 controllers.

It demonstrates how to set up a car-like robot using ROS 2 control, including configuration, launch files, and URDF integration.  
Most files are adapted directly from the official `ros2_control_demos` repository.

## Structure

- `urdf/` — Custom robot description files
- `config/` — Controller configuration
- `launch/` — Launch files for simulation and control
- `external/ros2_control_demos/` — Upstream demo files (as a submodule)

## Note

This project is intended for demonstration and experimentation purposes only. Please use ```git submodule update --init --recursive```. 
You can test this by 
```ros2 topic pub /carlikebot_base_controller/reference geometry_msgs/msg/TwistStamped "{
  twist: {
    linear: {x: 1.0, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.5}
  }
}" --rate 10```