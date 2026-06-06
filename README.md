# Industrial Arm Robot - ROS 2 & MoveIt 2

**Course:** Master 1 Robotics  
**Assignment:** Robot Arm Modeling with URDF/Xacro  
**GitHub:** [industrial-arm-assignment](https://github.com/akgooljar0709/industrial-arm-assignment)

---

## What This Project Does

This is a complete ROS 2 model of a 5-joint industrial robotic arm with a gripper. You can:
- Visualize the robot in RViz 2
- Control each joint with a GUI slider
- Plan and execute trajectories using MoveIt 2
- Simulate pick-and-place tasks

The robot has:
- **A rotating base** (joint1 - 360° rotation)
- **Two pitch joints** in the arm (joint2 & joint3 - up/down movement)
- **A wrist** that can rotate (joint4 - twist motion)
- **A parallel gripper** with two fingers that open/close together (joint5)

The whole chain looks like this:
```
World → Base Link → Base Plate → Forward Arm → Horizontal Arm → Gripper Support → Fingers
```

---

## Why I Chose Xacro Over Raw URDF

I went with **Xacro** (XML Macros) instead of writing one giant URDF file. Here's why:

### The Problem with Plain URDF
If I used plain XML, I'd have to copy-paste the same link and joint definitions over and over. For 7 links, that's a lot of repetition and it gets messy fast.

### Why Xacro Made Sense
- **Reusable macros:** I created an `inertial` macro so I don't repeat the mass/inertia math 7 times
- **Single source of truth:** Changed the mesh scale once (`scale = 0.01`) and it applied everywhere
- **Easier to read:** The structure is clearer with properties at the top
- **Future-proof:** If I add a 6th joint later, it's just changing parameters

### When I'd Use Plain URDF Instead
Honestly, for a super simple 2-joint robot that never changes, plain XML would be fine. But anything with repeated structures? Xacro wins.

---

## Robot Specifications

### The Links (7 parts)

| Part | Weight | What It Does | Collision |
|------|--------|--------------|-----------|
| base_link | 1.5 kg | Heavy base with motors | Cylinder |
| base_plate | 1.0 kg | Rotates on joint1 | Cylinder |
| forward_drive_arm | 0.8 kg | First arm segment | Box |
| horizontal_arm | 0.5 kg | Reaches out | Box |
| claw_support | 0.3 kg | Holds gripper | Box |
| gripper_right | 0.05 kg | Right finger | Box |
| gripper_left | 0.05 kg | Left finger (follows right) | Box |

**Note on weights:** I estimated these based on what makes sense for an aluminum robot with built-in motors. The base is heavier because it has all the drive electronics.

### The Joints (5 active)

| Joint | Type | Movement | Range | Force | Speed |
|-------|------|----------|-------|-------|-------|
| joint1 | Revolute | Base rotation | ±180° | 30 N·m | 2 rad/s |
| joint2 | Revolute | Arm pitch (up/down) | ±90° | 30 N·m | 2 rad/s |
| joint3 | Revolute | Elbow pitch | ±90° | 30 N·m | 2 rad/s |
| joint4 | Revolute | Wrist rotation | ±90° | 30 N·m | 2 rad/s |
| joint5_right | Prismatic | Gripper open/close | 0-22mm | 10 N | 0.5 m/s |
| joint5_left | Prismatic | Left finger (mirrors right) | 0-22mm | 10 N | 0.5 m/s |

The gripper fingers are **linked together** - when the right finger opens, the left automatically opens at the same speed.

---

## Getting It Running

### What You Need

- Ubuntu 22.04 or 24.04
- ROS 2 (Humble or Jazzy)
- colcon build tool
- MoveIt 2

### Step-by-Step Setup

**1. Install ROS 2 (if you don't have it)**

For Ubuntu 22.04:
```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-moveit
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

For Ubuntu 24.04:
```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-moveit
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

**2. Clone and Setup**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/akgooljar0709/industrial-arm-assignment.git
cd ~/ros2_ws
```

**3. Install Dependencies**

```bash
sudo apt install -y python3-colcon-common-extensions

rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**4. Build It**

```bash
colcon build --packages-select industrial_arm_description
source ~/ros2_ws/install/setup.bash
```

**5. Verify It Worked**

```bash
ros2 pkg list | grep industrial_arm_description
```

If you see `industrial_arm_description`, you're good!

---

## Running the Robot

### View the Robot in RViz

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch industrial_arm_description display.launch.py
```

This opens:
- **RViz window** - Shows your robot in 3D
- **Joint slider GUI** - Control each joint with sliders
- **Terminal** - Shows debug messages

Try moving the sliders around and watch the robot move in real-time!

### Use MoveIt for Motion Planning

First, generate the MoveIt config (see section below), then:

```bash
ros2 launch industrial_arm_description_moveit_config demo.launch.py
```

This lets you:
- Plan trajectories (calculate smooth paths)
- Simulate movements
- Visualize collision avoidance

---

## Setting Up MoveIt 2

MoveIt is for motion planning - calculating smooth, collision-free paths.

### Generate the Config Package

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```

**In the GUI that opens:**

1. **Load URDF** → Select `~/ros2_ws/src/industrial-arm-assignment/xacro/robot.urdf.xacro`
2. **Generate collision matrix** → Just click the button, defaults are fine
3. **Add planning group "arm"** 
   - Include joints: joint1, joint2, joint3, joint4
   - Solver: KDL (default)
4. **Add planning group "gripper"**
   - Include joints: joint5_right, joint5_left
5. **Set end effector** (optional but nice to have)
   - Name: gripper
   - End link: gripper_right
6. **Generate the package** → Pick output folder, click generate

After generation:

```bash
cd ~/ros2_ws
colcon build --packages-select industrial_arm_description_moveit_config
source ~/ros2_ws/install/setup.bash
```

Now you can launch MoveIt:

```bash
ros2 launch industrial_arm_description_moveit_config demo.launch.py
```

---

## File Structure

```
industrial-arm-assignment/
├── xacro/
│   └── robot.urdf.xacro          Main robot description file
├── meshes/
│   └── *.stl                      3D mesh files for each part
├── launch/
│   └── display.launch.py          Launch script for RViz
├── config/
│   └── display.rviz               RViz settings
├── package.xml                    Package info
├── CMakeLists.txt                 Build config
└── README.md                       This file
```

After MoveIt setup, you'll also get:
```
industrial_arm_description_moveit_config/
├── config/                        Planning parameters
├── launch/                        Launch files for MoveIt
└── package.xml
```

---

## Testing

### Check if the URDF is Valid

```bash
check_urdf ~/ros2_ws/src/industrial-arm-assignment/xacro/robot.urdf.xacro
```

You should see the full tree without errors.

### Test in RViz

```bash
ros2 launch industrial_arm_description display.launch.py
```

**What to look for:**
- Robot appears in the 3D view
- Joint sliders work and move the robot
- No red error messages in the console
- TF tree shows all frame connections

### Test Movements

Use the sliders to try:
- **joint1:** Rotate the base (full 360°)
- **joint2:** Lift the arm up/down
- **joint3:** Bend the elbow
- **joint4:** Twist the wrist
- **joint5_right/left:** Open and close gripper fingers (they move together)

---

## Issues I Hit & How I Fixed Them

### 1. Meshes Were Way Too Big

**Problem:** The STL files were way too large - the robot looked like a giant blob in RViz.

**Fix:** Applied `scale="0.01 0.01 0.01"` to shrink everything down. The assignment specifically mentioned this.

### 2. Robot Looked Twisted

**Problem:** Links were rotated and positioned wrong, the arm didn't look right.

**Fix:** Carefully applied all the origin transformations from the assignment PDF. Took a while to get them all right.

### 3. Gripper Fingers Weren't Synced

**Problem:** Left and right gripper fingers moved independently instead of together.

**Fix:** Added a `<mimic>` tag to make the left finger follow the right finger's movements.

### 4. Missing ROS Dependencies

**Problem:** Launch files failed because ROS packages weren't listed in package.xml.

**Fix:** Added all the dependencies: urdf, xacro, rviz2, joint_state_publisher_gui, etc.

### 5. RViz Couldn't Find Config File

**Problem:** RViz couldn't load the display settings on startup.

**Fix:** Created a proper `config/display.rviz` file and updated CMakeLists.txt to install it.

---

## Helpful Resources

- [ROS 2 Official Docs](https://docs.ros.org/en/humble/)
- [URDF Tutorial](https://docs.ros.org/en/humble/Tutorials/URDF/URDF-Main.html)
- [Xacro Reference](http://wiki.ros.org/xacro)
- [MoveIt 2 Setup](https://docs.ros.org/en/humble/Tutorials/Intermediate/Configuring-ROS2-with-MoveIt2/Setup-Assistant.html)
- [RViz2 Guide](https://docs.ros.org/en/humble/Concepts/Intermediate/About-RViz2.html)

## Quick Troubleshooting

| Error | Fix |
|-------|-----|
| `ros2: command not found` | Run: `source /opt/ros/humble/setup.bash` (or jazzy) |
| `Package not found` | Run: `source ~/ros2_ws/install/setup.bash` |
| `Can't find URDF` | Check the file path in launch script |
| `RViz won't open` | Install: `sudo apt install ros-humble-rviz2` |
| `Meshes look weird` | They're empty files - see the section about STL files |

---

**Author:** Robotics Student  
**License:** BSD-3-Clause  
**GitHub:** https://github.com/akgooljar0709/industrial-arm-assignment

Have fun with the robot! 🤖
