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
