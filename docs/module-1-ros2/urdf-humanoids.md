---
id: urdf-humanoids
title: "URDF for Humanoid Robots"
description: "Modeling links, joints, collisions, and semantics for humanoids in ROS 2."
sidebar_position: 4
---

# URDF for Humanoid Robots

## Beginner

URDF describes the robot’s physical structure. Links represent parts, joints define movement between parts. Visual and collision elements describe appearance and physics. Humanoid robots have many joints that must be defined correctly for control.

## Intermediate

Use XACRO to generate URDF from reusable macros. Provide accurate inertia and limits for stability. Add sensors and coordinate frames using `<origin>` tags. Ensure collision geometry is simple to keep simulation fast while preserving safety.

## Advanced

Define consistent frames for hands, feet, and head to integrate perception and manipulation. Tune joint limits, damping, and effort for controllers. Use convex hulls for collision meshes and separate visual meshes for fidelity. Validate semantics with MoveIt 2 and kinematic solvers.

## ROS 2 Code Examples

```xml
<robot name="humanoid_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
  </link>
  <link name="shoulder_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="30.0" velocity="2.0"/>
  </joint>
</robot>
```

```bash
ros2 run gazebo_ros spawn_entity.py -file humanoid_arm.urdf -entity arm
```

## Humanoid-Specific Explanations

- Hands require precise frames for grasp planning.
- Feet frames must align with ground contact and balance algorithms.
- Head cameras need stable frames relative to torso for perception.

## Exercises

- Build a forearm URDF with accurate joint limits and test in Gazebo.
- Add collision geometry and validate with contact debugging.
- Integrate a simple camera link and verify the TF tree.

## Assessments

- Multiple Choice: Which element defines physics collision boundaries?
- Short Answer: Why separate visual and collision meshes for humanoids?
- Scenario: The arm oscillates in simulation. Which URDF parameters should be tuned?

## RAG Summary Chunks

- Chunk 1: URDF defines links, joints, visuals, and collisions for robot structure.
- Chunk 2: Humanoids need accurate frames for hands, feet, and head.
- Chunk 3: XACRO, limits, and inertia tuning improve stability and performance.



