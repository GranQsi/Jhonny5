# Jhonny5
# Robot Control System with ROS 2 and Micro-ROS

This repository provides all the necessary files and instructions for setting up a robot using ROS 2 Humble, Micro-ROS for Raspberry Pi Pico, and the necessary configurations for motor control, QR code processing, and PID control for centering the camera.

## Prerequisites

1. Raspberry Pi 4 with Ubuntu 22.04
2. ROS 2 Humble
3. Raspberry Pi Pico with Micro-ROS
4. Additional sensors: QR Code reader, motor encoders, and a camera

## Installation

Follow the steps below to set up the environment and firmware for the robot.

### Step 1: Install ROS 2 Humble on Raspberry Pi 4

Follow the official instructions to install ROS 2 Humble on your Raspberry Pi 4:
[ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Step 2: Install Micro-ROS on Raspberry Pi Pico

Install the necessary tools and firmware for Micro-ROS on Raspberry Pi Pico:

1. Clone and build Micro-ROS Setup repository:
   ```bash
   git clone https://github.com/micro-ROS/micro_ros_setup.git
   cd micro_ros_setup
   rosdep install --from-paths src --ignore-src -y
   colcon build
   source install/setup.bash

## Running the Jhonny5
Robot Johnny 5

Wifi:	xxxx

Pas:	************

ssh xxx@robot.local

pass: @@@@@@

### INTO THE RASPI -  MICROROS
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200

Después conectar el micro y publicar la odometría

cd ros2_ws

source install/local_setup.bash 

ros2 launch robot robot_base.launch.py

### INTO PC

enable slam:

ros2 launch robot robot_slam.launch.py 

enable rviz:

ros2 run rviz2 rviz2

(add topic lidar y map)


# manage the robot

ros2 run teleop_twist_keyboard teleop_twist_keyboard

install servermap

sudo apt install ros-humble-nav2-bringup 

save the map (my_map.png / my_map.yaml)

ros2 run nav2_map_server map_saver_cli -f my_map


