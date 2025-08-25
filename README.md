# ROS Turtlesim Teleoperation - Autonomous Systems

This project implements a custom teleoperation node for controlling a turtle in the ROS turtlesim simulator using keyboard input. The project was developed using VMware Workstation running Linux and follows ROS best practices for package creation and node development.

## Project Overview

The `autoturtle` ROS package provides keyboard-based control for a turtle in the turtlesim environment. Users can control the turtle's movement using WASD keys in real-time through a Python-based ROS node.

## System Requirements

### Host System
- VMware Workstation
- Windows host system

### Virtual Machine
- Linux distribution
- ROS Melodic/Noetic installation
- Python 3.x
- Required Python libraries: `curses`

## Installation and Setup

### Step 1: Setup Virtual Environment

1. **Install VMware Workstation** on your host system
2. **Create a Linux Virtual Machine** with Ubuntu 18.04 or 20.04
3. **Install ROS** following the official ROS installation guide

### Step 2: ROS Installation

I followed this nice tutorial series for ROS setup and package creation:
[ROS Tutorials Playlist](https://www.youtube.com/playlist?list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q)

### Step 3: Workspace Setup

1. **Clone or copy this project** to your Linux VM
2. **Build the workspace**:
   ```bash
   catkin_make
   ```
3. **Source the workspace**:
   ```bash
   source devel/setup.bash
   ```

### Step 4: Package Dependencies

The `autoturtle` package depends on:
- `geometry_msgs` - For Twist messages (velocity commands)
- `rospy` - Python ROS client library
- `std_msgs` - Standard ROS message types
- `turtlesim` - Turtle simulation environment

These dependencies are automatically handled during the build process.

## Usage Instructions

### Running the Teleoperation Demo

1. **Start ROS Master**:
   ```bash
   roscore
   ```

2. **Launch Turtlesim Node** (in a new terminal):
   ```bash
   source devel/setup.bash
   rosrun turtlesim turtlesim_node
   ```

3. **Run the Teleoperation Node** (in another new terminal):
   ```bash
   source devel/setup.bash
   rosrun autoturtle my_teleop_node.py
   ```

### Controls

Once the teleoperation node is running, use the following keys to control the turtle:

- **W** - Move forward
- **S** - Move backward  
- **A** - Turn left (counterclockwise)
- **D** - Turn right (clockwise)
- **Q** - Quit the program

The current action will be displayed on the terminal screen.

### Movement Parameters

- **Linear velocity**: ±0.3 m/s (forward/backward)
- **Angular velocity**: ±0.5 rad/s (left/right rotation)
- **Update rate**: 10 Hz