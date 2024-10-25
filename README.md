# Broverette

https://github.com/user-attachments/assets/03323889-74ca-40c9-b6ba-a75158bd82f6

## Overview
Broverette is a ROS 2 robot platform that handles core functionalities such as `cmd_vel`, `scan`, `odom`, and more. This repository contains the ROS 2 packages needed to control Broverette and interface with its sensors.

## Features
- Handles the core control topics for the Broverette robot.
- Interfaces with various sensors (LiDAR, IMU, Camera, etc.).
- Publishes essential topics for robot state, velocity, and sensor data.

### Required Components
You can find all the necessary parts in this Amazon list:
[Amazon List for Broverette](https://www.amazon.com/hz/wishlist/ls/1HPBW0ZJIIN79?ref_=wl_share)


## ROS Nodes
### Available Nodes
- **/LD19**: LiDAR node.
- **/base_link_to_base_laser_ld19**: Transform node for LiDAR to base link.
- **/battery_display**: Publishes battery voltage information.
- **/odom_pub**: Publishes odometry data.
- **/robot_state_publisher**: Publishes robot state.
- **/usb_cam**: Camera node.
- **/vel_raw_pub**: Publishes raw velocity data.

## Installation
### Prerequisites
- ROS 2 Humble
- A Raspberry Pi 4 With Ubuntu 22.04
- Colcon build system
  
From this link, you can download the **image (img)** file to flash onto the microSD card for the Raspberry Pi 4B 8GB. I recommend using [Etcher](https://etcher.balena.io/) for flashing the image.

#### **Link**:
[Ubuntu 22.04 Server Pi 4 IMG](https://drive.google.com/drive/folders/1WxypJMW6T0hi-66JYA1BB3ZOif5wu8fc?usp=sharing)

*I will make every effort to update the code in the image whenever changes are made. If for any reason I am unable to update it immediately, I will mention it here so that you can clone the latest version of the repository and rebuild accordingly.*

## Update the Wi-Fi on the Pi
Connect the Raspberry Pi to a monitor and sign in using the following credentials:

- Username: `ubuntu`
- Password: `123`

### Steps to Update or Configure Wi-Fi Using `nmtui`
   1.**Open Terminal**: Start by opening a terminal on your system.
   
   2.**Launch nmtui**: In the terminal, type the following command to launch the Network Manager Text User Interface:
```bash
nmtui
```
   3.**Navigate the Menu**: Once the nmtui interface opens, you’ll be presented with three options:
- Activate a connection
- Set system hostname
- Edit a connection

Use the arrow keys to select **Activate a connection** and press **Enter**.

If only one connection is initially displayed, press **Enter** to load the rest. Then, use the arrow keys to select the desired network and enter the password.

### Clone the Repository and Set Up the Workspace

1. **Create a new workspace**:
   First, create a workspace for ROS 2 if you don’t have one already:
   ```bash
   mkdir ~/broverette_ws/
   cd ~/berette_ws/
   ```
2. **Clone the repository into the workspace**:
   ```bash
   git clone https://github.com/thetacobytes/Broverette.git .
   ```
3. **Go to the workspace root and build**:
   Navigate back to the workspace root and build the workspace using `colcon`:
   ```bash
   cd ~/broverette_ws
   colcon build
   ```
4. **Source the workspace**:
   After building, source the setup file:
   ```bash
   source install/setup.bash
   ```
### Running the Nodes
You can now run the various ROS 2 nodes provided in this workspace with one launch command.
   ```bash
   ros2 launch broverette_description start_nodes_launch.py
  ```
### Setting the `ROS_DOMAIN_ID` for Multi-Machine Communication
If you are using multiple machines and want them to see each other's topics, you need to set the `ROS_DOMAIN_ID` to the same value on all machines. For example, you can set it to `1` by running:
   ```bash
   export ROS_DOMAIN_ID=1
   ```
# Note

This repository contains the core description and configuration files for the Broverette robot, but **it does not include any control mechanisms or navigation capabilities**.

## Controlling Broverette and Creating Maps

If you want to control the Broverette using a PS4 controller, perform **SLAM** to create maps, or use **Nav2** and **AMCL** for autonomous navigation, please refer to the Broverette Navigation repository:

[**Broverette Navq Repository**](https://github.com/TheTacoBytes/Broverette_Nav2)

The **Broverette Navigation** repository is designed to be run on a laptop that communicates with the Pi 4 on the Broverette. This will run:

- The PS4 controller node to control the robot manually.
- SLAM to create maps of the environment.
- AMCL and Nav2 for path planning and navigation.
- USB Camera feed for video streaming.

Be sure to clone the **Broverette Navigation** repository to your **laptop** to perform these actions.
