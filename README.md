# Industrial Arm ROS 2 Project

This repository contains the URDF/Xacro description for an industrial robotic arm, developed and validated for ROS 2.

## Project Overview
This package defines the kinematic structure of an industrial arm with 5 degrees of freedom, including a parallel gripper mechanism. The model has been verified using ROS 2 native parsing tools.

## Kinematic Structure
The robot follows the following transformation hierarchy:
- **world** -> **base_link** -> **base_plate**
- **base_plate** -> **forward_drive_arm**
- **forward_drive_arm** -> **horizontal_arm**
- **horizontal_arm** -> **claw_support**
- **claw_support** -> **gripper_left** & **gripper_right** (parallel mimic)

## Verification
The URDF was validated using the `check_urdf` utility:
```text
Robot Name: industrial_arm
---------- Successfully Parsed XML ---------------
Root Link: world has 1 child(ren)
    child(1):  base_link
        child(1):  base_plate
            child(1):  forward_drive_arm
                child(1):  horizontal_arm
                    child(1):  claw_support
                        child(1):  gripper_left
                        child(2):  gripper_right


# How to Build

## 1. Clone the Repository

Clone this repository into the `src` folder of your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/akgooljar0709/industrial-arm-assignment.git
```

## 2. Build the Package

Build the package using `colcon`:

```bash
colcon build --packages-select industrial_arm_description
```

## 3. Source the Workspace

After the build completes successfully, source the workspace setup file:

```bash
source install/setup.bash
```

## 4. Verify the Installation (Optional)

Confirm that the package is available in your ROS 2 environment:

```bash
ros2 pkg list | grep industrial_arm_description
```
