---
sidebar_position: 3
id: nav2-vslam
title: "Isaac ROS VSLAM & Nav2"
description: "Hardware-accelerated Visual SLAM and navigation for bipedal humanoids."
---

# Isaac ROS VSLAM & Nav2

## 1. Learning Objectives
By the end of this chapter, you will be able to:
- **Explain** the concept of Visual SLAM (Simultaneous Localization and Mapping).
- **Configure** Isaac ROS VSLAM for hardware-accelerated localization.
- **Understand** how Nav2 (Navigation 2) plans paths for robots.
- **Adapt** Nav2 parameters for bipedal humanoid movement constraints.

---

## 2. Core Explanation (Intermediate)

### Visual SLAM (VSLAM)
For a robot to move, it must answer two questions: "Where am I?" (Localization) and "What does the world look like?" (Mapping).
**Visual SLAM** uses cameras to track feature points (corners, edges) in the environment to estimate the robot's position and build a map simultaneously.

**Isaac ROS VSLAM**:
- Uses **GPU acceleration** (Elbrus library) to perform tracking at high frame rates (30+ FPS) on Jetson hardware.
- More robust than standard CPU-based SLAM (like ORB-SLAM) in dynamic environments.
- Outputs the robot's pose (`/tf`) relative to the map frame.

### Nav2 for Humanoids
**Nav2** is the standard navigation stack for ROS 2. It takes the robot's position (from VSLAM) and a goal pose, then generates velocity commands (`cmd_vel`).

**Challenges with Humanoids**:
- unlike wheeled robots, humanoids sway when walking.
- They cannot turn in place instantly (unless using specific gait).
- **Footprint**: The robot's collision shape changes as it moves legs.

We use **Nav2 Controller Plugins** (like MPPI or Regulated Pure Pursuit) tuned for the specific kinematics of the humanoid to ensure smooth path following.

---

## 3. Beginner Simplification

**The Explorer**:
Imagine you are dropped in a dark room with a flashlight.
1.  **VSLAM**: You look around and remember "The door is there, the table is there." As you move, you count your steps to know where you are.
2.  **Nav2**: You want to go to the fridge. You plan a path: "Walk forward 5 steps, turn left, walk 3 steps."
3.  **Isaac ROS**: This is like having a photographic memory and a super-fast brain that processes what you see instantly, so you never get lost.

---

## 4. Advanced Deep-Dive

### Hardware Acceleration
Isaac ROS VSLAM runs on the **NVIDIA PVA (Programmable Vision Accelerator)** and **GPU** on Jetson Orin modules. This offloads the CPU, leaving it free for the high-frequency control loops required for humanoid balance.

### Nav2 Architecture
- **Global Planner**: Calculates the path from A to B (e.g., Dijkstra or A*).
- **Local Planner (Controller)**: Follows the path while avoiding dynamic obstacles (people, pets).
- **Costmaps**: 2D or 3D grids representing safe and unsafe areas. For humanoids, we often use **Voxel Layers** to detect overhanging obstacles (like tables) that the head might hit.

---

## 5. Code Examples

### Launching Isaac ROS VSLAM
```python
# launch_vslam.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam_node',
            parameters=[{
                'enable_image_denoising': True,
                'rectified_images': True,
                'enable_imu_fusion': True
            }],
            remappings=[
                ('stereo_camera/left/image_rect', '/camera/left/image_rect'),
                ('stereo_camera/right/image_rect', '/camera/right/image_rect'),
                ('stereo_camera/left/camera_info', '/camera/left/camera_info'),
                ('stereo_camera/right/camera_info', '/camera/right/camera_info')
            ]
        )
    ])
```

---

## 6. Real-World Context
**Warehouse Logistics**:
Robots like **Agility Robotics' Digit** use VSLAM to navigate warehouses where GPS doesn't work. They use Nav2 to plan paths around boxes and forklifts.

---

## 7. Exercises
1.  **Simulation**: Launch Isaac Sim with a simple warehouse environment.
2.  **VSLAM**: Run the Isaac ROS VSLAM node and visualize the generated map in RViz.
3.  **Navigation**: Send a "2D Nav Goal" in RViz and watch the robot plan a path to that location.

---

## 8. Assessment Questions
1.  **Explain**: Why is VSLAM preferred over GPS for indoor humanoid robots?
2.  **Debug**: If the robot thinks it is moving backwards when it is moving forwards, what sensor data might be inverted? (IMU or Wheel Odometry).
3.  **Design**: How would you configure Nav2 for a robot that is very tall but thin? (Adjust the footprint and costmap height).
