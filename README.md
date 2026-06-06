# Industrial Arm ROS 2 Robot Description & MoveIt 2 Configuration

**Course:** Master 1 - Robotique & Systèmes embarqués  
**Project:** Modélisation d'un bras robotique - URDF/Xacro & MoveIt 2  
**Repository:** [GitHub - industrial-arm-assignment](https://github.com/akgooljar0709/industrial-arm-assignment)

---

## 1. Project Overview

This project presents a complete ROS 2 implementation of a 5-DOF industrial robotic arm with a parallel gripper mechanism. The work encompasses:

- **Robot Kinematics:** Full URDF/Xacro description with 7 links and 5 joints
- **Visualization:** RViz 2 integration with interactive joint control
- **Motion Planning:** MoveIt 2 configuration for trajectory planning and execution
- **Collision Detection:** Simplified collision geometry for planning and safety

The robot consists of:
- **Base mechanism:** Fixed base_link and rotating base_plate (joint1)
- **Arm structure:** 
  - forward_drive_arm (joint2: pitch movement)
  - horizontal_arm (joint3: pitch movement)
  - claw_support (joint4: yaw movement)
- **Gripper:** Parallel gripper with left/right fingers (joint5_left and joint5_right)
  - Mimic joint implementation for synchronized opening/closing

### Kinematic Chain

```
world
  └─ [virtual_joint: fixed]
     └─ base_link (visual + collision)
        └─ [joint1: revolute Z-axis]
           └─ base_plate (visual + collision)
              └─ [joint2: revolute Y-axis]
                 └─ forward_drive_arm (visual + collision)
                    └─ [joint3: revolute Y-axis]
                       └─ horizontal_arm (visual + collision)
                          └─ [joint4: revolute Z-axis]
                             └─ claw_support (visual + collision)
                                ├─ [joint5_right: prismatic X-axis]
                                │  └─ gripper_right (visual + collision)
                                └─ [joint5_left: prismatic X-axis + mimic]
                                   └─ gripper_left (visual + collision)
```

---

## 2. Architecture Choice: URDF vs Xacro

### Decision: **Xacro (Modular Parametric Format)**

#### Justification

We selected **Xacro** (XML Macros) over a single monolithic URDF file based on the following criteria:

| Criterion | Single URDF | Xacro Modularity | **Decision** |
|-----------|-------------|------------------|------------|
| **Complexity** | 5 DOF requires many repetitive link/joint definitions | Macros reduce repetition by ~40% | ✅ Xacro |
| **Reusability** | Links/joints cannot be reused | Macro-based components are reusable | ✅ Xacro |
| **Maintainability** | Large monolithic file (>300 lines) | Organized with properties and macros | ✅ Xacro |
| **Scalability** | Adding 6th DOF requires full restructuring | Simple macro parameter adjustment | ✅ Xacro |
| **Readability** | Difficult to follow transformation chain | Clear macro definitions with parameters | ✅ Xacro |
| **Parametrization** | Hardcoded values throughout | Central property definitions (PI, scales) | ✅ Xacro |

#### Advantages of Our Xacro Approach

1. **Inertial Macro (`box_inertial`)**: Eliminates repetitive inertia tensor calculations
2. **Mesh Scaling Property**: Single `mesh_scale = 0.01` definition applies to all 7 meshes
3. **Joint Limits Property**: Centralized effort/velocity parameters for consistency
4. **Conditional Compilation**: Supports `use_sim_time` parameter for Gazebo integration
5. **Extensibility**: Easy to add gripper fingers, sensors, or additional DOF

#### When We Would Use Single URDF

- For very simple robots (≤2 DOF)
- When parameters never change
- In embedded systems with strict XML-only parsing requirements
- For one-time static descriptions without future modifications

---

## 3. Robot Description Details

### 3.1 Links (7 total)

| Link | Mass (kg) | Purpose | Collision Type | Notes |
|------|-----------|---------|------------------|-------|
| **world** | 0 | Root TF frame | None | Virtual reference |
| **base_link** | 1.5 | Base platform | Cylinder (r=0.1m) | Heavy due to motors |
| **base_plate** | 1.0 | Rotating platform | Cylinder (r=0.08m) | Mounted on joint1 |
| **forward_drive_arm** | 0.8 | First arm segment | Box (0.04×0.04×0.1m) | Lighter aluminum |
| **horizontal_arm** | 0.5 | Second arm segment | Box (0.12×0.03×0.03m) | Long reach member |
| **claw_support** | 0.3 | Gripper mounting | Box (0.03×0.05×0.04m) | Minimal mass |
| **gripper_right/left** | 0.05 each | Gripper fingers | Box (0.02×0.01×0.02m) | Lightweight aluminum |

### Mass & Inertia Assumptions

**Methodology:**
- Base assemblies (base_link, base_plate): Higher mass due to motor/gearbox integration
- Arm segments: Progressive mass reduction toward end-effector
- Gripper: Minimal mass (servo-driven)

**Inertia Calculation:**
Used rectangular box inertia tensor (conservative estimate):
$$I_{xx} = \frac{1}{12}m(y^2 + z^2), \quad I_{yy} = \frac{1}{12}m(x^2 + z^2), \quad I_{zz} = \frac{1}{12}m(x^2 + y^2)$$

These are estimates based on aluminum construction with integrated electronics.

### 3.2 Joints (5 active + 1 virtual)

| Joint | Type | Axis | Limits | Effort | Velocity | Purpose |
|-------|------|------|--------|--------|----------|---------|
| **virtual_joint** | fixed | - | - | - | - | World-to-base attachment |
| **joint1** | revolute | Z | ±π rad | 30 N·m | 2 rad/s | Base rotation (yaw) |
| **joint2** | revolute | Y | ±π/2 rad | 30 N·m | 2 rad/s | Arm pitch (lift) |
| **joint3** | revolute | Y | ±π/2 rad | 30 N·m | 2 rad/s | Elbow pitch (reach) |
| **joint4** | revolute | Z | ±π/2 rad | 30 N·m | 2 rad/s | Wrist yaw (roll) |
| **joint5_right** | prismatic | X | [0, 0.022m] | 10 N | 0.5 m/s | Right gripper open/close |
| **joint5_left** | prismatic | X | [0, 0.022m] | 10 N | 0.5 m/s | Left gripper (mimic right) |

**Joint Limits Reasoning:**
- Revolute joints: ±π (full rotation) or ±π/2 (90° range) based on mechanical constraints shown in assignment
- Prismatic joints: 22mm range matches gripper finger geometry from STL files
- Effort values: Conservative 30 N·m for arm joints, 10 N for gripper (sufficient for pick-and-place)

### 3.3 Visual & Collision Elements

**Mesh Scaling:** All STL files are scaled by factor of 0.01 in X, Y, Z (as specified in assignment)

**Origin Transformations:** Applied exactly as provided in assignment documentation:
- world→base_link: origin rpy="0 0 0" xyz="-0.5 -0.5 0"
- base_link→base_plate: origin rpy="0 0 0" xyz="-0.39 -0.39 -0.56"
- etc. (see xacro file for complete list)

**Collision Simplification:**
- Cylindrical collisions for rotating bases (joint rotation efficiency)
- Box collisions for arm segments (computation speed)
- Simplified gripper boxes instead of complex finger geometry

---

## 4. Installation & Setup

### Prerequisites

- **OS:** Ubuntu 22.04 LTS or 24.04 LTS
- **ROS 2:** Humble or Jazzy distribution
- **Build tool:** colcon
- **Required packages:**
  - ros2-humble-desktop (or jazz-equivalent)
  - ros2-humble-moveit
  - ros2-humble-rviz2

### 4.1 Install ROS 2

If not already installed:

```bash
# Ubuntu 22.04 (Humble)
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-moveit

# Ubuntu 24.04 (Jazzy)
sudo apt install -y ros-jazzy-desktop ros-jazzy-moveit
```

### 4.2 Setup ROS 2 Workspace

```bash
# Create workspace (if not already exists)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/akgooljar0709/industrial-arm-assignment.git

# Navigate to workspace root
cd ~/ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select industrial_arm_description

# Source the workspace
source install/setup.bash
```

### 4.3 Verify Installation

```bash
# Check if package is found
ros2 pkg list | grep industrial_arm_description

# Validate URDF/Xacro
check_urdf industrial_arm_description/robot.urdf.xacro

# List available launch files
ros2 launch industrial_arm_description --help
```

---

## 5. Running the Robot

### 5.1 Launch RViz 2 with Interactive Control

Visualize the robot and control joints with a GUI:

```bash
ros2 launch industrial_arm_description display.launch.py
```

This will:
- Spawn RViz 2 window with the robot model
- Launch joint_state_publisher_gui for interactive joint control
- Display the TF tree and robot state

**Usage in RViz:**
- Use the slider GUI window to move individual joints
- Observe the robot model update in real-time
- Verify collision geometries are correct

### 5.2 Launch MoveIt 2 (After Setup Assistant)

Once the MoveIt config package is generated (see section 6):

```bash
ros2 launch industrial_arm_description_moveit_config demo.launch.py
```

This provides:
- Motion planning interface
- Trajectory visualization
- Goal pose selection and planning

---

## 6. MoveIt 2 Configuration & Setup

### 6.1 Generate MoveIt Configuration

The MoveIt Setup Assistant generates the configuration package. Run:

```bash
# Terminal 1: Start MoveIt Setup Assistant
ros2 run moveit_setup_assistant moveit_setup_assistant

# In GUI:
# 1. Select URDF file: ~/ros2_ws/src/industrial-arm-assignment/xacro/robot.urdf.xacro
# 2. Auto-add links from URDF
# 3. Configure planning groups:
#    - Group name: "arm"
#    - Select joints: joint1, joint2, joint3, joint4
#    - IK solver: KDL
#    - Group name: "gripper"
#    - Select joints: joint5_right, joint5_left
# 4. Define end effector:
#    - Name: "gripper"
#    - Attached links: gripper_right, gripper_left
# 5. Configure collision matrix (auto-generate)
# 6. Configure controllers (load from template)
# 7. Author info and license
# 8. Generate package → install/
```

### 6.2 Expected Output Structure

After generation, you should have:

```
industrial_arm_description_moveit_config/
├── config/
│   ├── arm.srdf
│   ├── cartesian_constraints.yaml
│   ├── controllers.yaml
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── moveit.rviz
│   └── moveit_controllers.yaml
├── launch/
│   ├── demo.launch.py
│   ├── move_group.launch.py
│   ├── rviz.launch.py
│   └── rsp.launch.py
├── package.xml
├── CMakeLists.txt
└── README.md
```

### 6.3 Verify MoveIt Configuration

```bash
# Build the generated package
colcon build --packages-select industrial_arm_description_moveit_config

# Check SRDF validity
ros2 launch industrial_arm_description_moveit_config demo.launch.py

# In RViz:
# - Select planning group "arm"
# - Set start/goal poses
# - Click "Plan" to visualize trajectory
# - Click "Execute" to simulate motion (if not in real hardware)
```

---

## 7. Project Structure

```
industrial-arm-assignment/
├── xacro/
│   └── robot.urdf.xacro          ← Main Xacro robot description
├── meshes/
│   ├── base_link.stl
│   ├── base_plate.stl
│   ├── forward_drive_arm.stl
│   ├── horizontal_arm.stl
│   ├── claw_support.stl
│   ├── gripper_right.stl
│   └── gripper_left.stl
├── launch/
│   └── display.launch.py          ← RViz visualization launch
├── config/
│   └── display.rviz               ← RViz configuration
├── package.xml                    ← ROS 2 package metadata
├── CMakeLists.txt                 ← Build configuration
└── README.md                       ← This file

(After MoveIt Setup Assistant:)
industrial_arm_description_moveit_config/
├── config/
│   ├── arm.srdf
│   └── ... (other MoveIt configs)
├── launch/
│   ├── demo.launch.py
│   └── ... (other launch files)
├── package.xml
└── CMakeLists.txt
```

---

## 8. Testing & Validation

### 8.1 URDF Validation

```bash
# Parse and check URDF syntax
ros2 run urdf_parser_py check_urdf xacro/robot.urdf.xacro

# View the kinematic tree
ros2 run urdf_parser_py display_urdf xacro/robot.urdf.xacro
```

### 8.2 TF Tree Verification

In RViz, enable the "TF" display to visualize the transformation tree:
- All frames should be connected
- No red errors in the TF panel
- Frame hierarchy matches kinematic chain

### 8.3 Joint Movement Verification

Using `joint_state_publisher_gui`:
- Move joint1 (base rotation): Should rotate ±180°
- Move joint2 (arm pitch): Should pitch ±90°
- Move joint3 (elbow pitch): Should pitch ±90°
- Move joint4 (wrist yaw): Should rotate ±90°
- Move joint5_right/left: Gripper fingers should open/close in sync

### 8.4 Collision Detection

Enable collision visualization in RViz:
1. Open display options
2. Enable "RobotModel" → "Collision Enabled"
3. Move joints and verify no self-collisions

---

## 9. Difficulties Encountered & Solutions

### 9.1 Mesh Scaling Issues

**Problem:** STL meshes appeared 100x too large in RViz

**Solution:** Applied `scale="0.01 0.01 0.01"` to all mesh geometry tags as specified in assignment. This scales the 1mm-resolution meshes to proper centimeter scale.

### 9.2 Joint Origin Transformations

**Problem:** Robot appeared distorted, links not aligned

**Solution:** Carefully applied each origin transformation from the assignment PDF:
- World→base_link: xyz="-0.5 -0.5 0"
- base_plate→forward_drive_arm: rpy="0 -π/2 π/2"
- etc.

Verified alignment by comparing RViz rendering with assignment reference images.

### 9.3 Gripper Mimic Joint

**Problem:** Gripper fingers not synchronized

**Solution:** Added mimic joint tag:
```xml
<mimic joint="joint5_right" multiplier="1"/>
```
This makes joint5_left follow joint5_right with 1:1 ratio for synchronized opening/closing.

### 9.4 Package.xml Dependencies

**Problem:** Launch files failed due to missing dependencies

**Solution:** Added all required dependencies:
- `urdf`, `xacro` (description)
- `joint_state_publisher_gui`, `robot_state_publisher` (visualization)
- `rviz2` (display)
- `moveit_core`, `moveit_ros_planning_interface` (planning)

### 9.5 Launch File Configuration

**Problem:** RViz couldn't find display configuration

**Solution:** 
1. Created dedicated `config/` directory
2. Generated `display.rviz` with proper panel/display settings
3. Updated CMakeLists.txt to install config directory
4. Used `LaunchConfiguration` in launch script for flexible path resolution

---

## 10. References & Resources

- [ROS 2 URDF Documentation](https://docs.ros.org/en/humble/Tutorials/URDF/URDF-Main.html)
- [Xacro Macro System](http://wiki.ros.org/xacro)
- [MoveIt 2 Setup Assistant](https://docs.ros.org/en/humble/Tutorials/Intermediate/Configuring-ROS2-with-MoveIt2/Setup-Assistant.html)
- [RViz 2 Visualization](https://docs.ros.org/en/humble/Concepts/Intermediate/About-RViz2.html)
- [TF2 Transform Library](https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Tf2-Main.html)

---

## 11. Author & License

**Author:** Robotics Team - Master 1, Robotique & Systèmes embarqués  
**License:** BSD-3-Clause  
**Repository:** https://github.com/akgooljar0709/industrial-arm-assignment

For questions or issues, please open an issue on GitHub.
