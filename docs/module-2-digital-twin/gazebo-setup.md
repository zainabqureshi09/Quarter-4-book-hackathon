---
id: gazebo-setup
title: "Gazebo Environment Setup"
description: "Install, launch, spawn entities, and integrate ROS 2."
sidebar_position: 3
---

# Gazebo Environment Setup

## Beginner

Install Gazebo and launch a basic world. Spawn a simple robot model and verify it appears. Use ROS 2 tools to connect nodes and visualize topics.

## Intermediate

Configure a custom world with lighting and ground plane. Use `gazebo_ros` to bridge topics and services. Spawn URDF or SDF models via CLI and set namespaces to organize humanoid subsystems.

## Advanced

Run headless simulations for CI. Use ROS 2 parameters to configure plugins at runtime. Launch multiple worlds and robots with namespaces. Profile real-time factor and use `gz` tools to inspect performance.

## Gazebo Configuration Example

```bash
ros2 launch gazebo_ros gazebo.launch.py
ros2 run gazebo_ros spawn_entity.py -file humanoid_arm.urdf -entity arm -x 0 -y 0 -z 1
ros2 topic list
```

```xml
<sdf version="1.9">
  <world name="flat_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

## Humanoid Context

- Use namespaces per limb and sensor to avoid collisions.
- Spawn controllers after the robot to ensure parameter availability.
- Bridge joint states and TFs to ROS 2 for MoveIt 2.

## Exercises

- Launch a world and spawn two robots in separate namespaces.
- Inspect topics and verify joint state publishing.
- Run headless and measure performance changes.

## Assessments

- Multiple Choice: Which tool spawns a URDF into Gazebo?
- Short Answer: Why use namespaces in multi-robot setups?
- Scenario: Topics collide between robots. What configuration fix applies?

## RAG Summary Chunks

- Chunk 1: Gazebo launches worlds and spawns URDF/SDF models.
- Chunk 2: `gazebo_ros` bridges topics and services into ROS 2.
- Chunk 3: Namespaces organize humanoid subsystems and avoid conflicts.



