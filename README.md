# Automated Blood Sample Handling Robot

## Overview
This project introduces an Automated Blood Sample Handling Robot designed to streamline the process of picking blood sampling slits and delivering them to testing equipment. The robot is equipped with a UR10 Robotics arm and a vacuum gripper for efficient and precise sample handling.

## ROS2 Package - UR10
The project includes the ROS2 Package named ur10. The package provides the control scripts for a UR10 Robotics arm equipped with vaccumm gripper, which is designed for the task of picking blood sampling slits to the testing equipment. The scripts uses the inverse kinematics approach to perform the end-effector movement.

### Disclaimer
It's important to note that the presented environment is a simulation, not the actual work environment. The simulation is created to represent real-world scenarios, where, for instance, a cafe table is replaced by the testing equipment, and the cardboard box (representing a small 5cm x 5cm box) is substituted with the table for blood sample placement.



## Getting Started

1. Clone this repository: git clone : https://github.com/tathya7/Automated-Blood-Sample-Handling-Robot.git
3. Navigate to the project directory: cd automated-blood-sample-robot
4. Install dependencies: rosdep install --from-paths src --ignore-src -r -y
5. Build the workspace: colcon build
6. Source the setup file: source install/setup.bash
7. Launch the robot control node: ros2 launch ur10 robot_control.launch.py
