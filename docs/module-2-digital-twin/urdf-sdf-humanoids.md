---
id: urdf-sdf-humanoids
title: "URDF & SDF for Humanoid Robots"
description: "Modeling humanoids with URDF/XACRO and SDF worlds for simulation."
sidebar_position: 4
---

# URDF & SDF for Humanoid Robots

## Beginner

URDF defines robot parts and movements. SDF defines the simulation world. Together they describe how a humanoid looks and moves inside the simulator.

## Intermediate

Use XACRO to reuse macros for repeated limbs. Provide accurate limits, inertias, and frames. Embed sensors and controllers via tags and plugins. Use SDF to configure world physics and include your robot model.

## Advanced

Separate visual and collision meshes. Use convex hulls for collisions. Tune joint damping and effort for stable control. Validate kinematics with MoveIt 2 and ensure consistent frames for manipulation and perception tasks.

## URDF Example

```xml
<robot name="humanoid_leg">
  <link name="hip"/>
  <link name="thigh"/>
  <joint name="hip_joint" type="revolute">
    <parent link="hip"/>
    <child link="thigh"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.5" upper="1.5" effort="50.0" velocity="2.0"/>
  </joint>
</robot>
```

## SDF Example

```xml
<sdf version="1.9">
  <world name="humanoid_world">
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

- Legs require precise frames and limits for gait generation.
- Arms need consistent hand frames for grasp planning.
- Torso and head frames align sensors and controllers for stability.

## Exercises

- Create a XACRO macro for a finger and instantiate five fingers.
- Add collision meshes and test contacts on the ground.
- Build a MoveIt 2 configuration and validate kinematics.

## Assessments

- Multiple Choice: Which format defines the simulation world?
- Short Answer: Why use convex hulls for collision meshes?
- Scenario: Grasp planning fails due to frame mismatch. What should be fixed?

## RAG Summary Chunks

- Chunk 1: URDF defines robot links and joints; SDF defines world and physics.
- Chunk 2: XACRO macros enable reuse for humanoid limbs.
- Chunk 3: Collision mesh tuning improves stability and performance.

## Urdu Translation

- English: URDF robot ki banaawat aur harkat ko bayan karta hai.
- Roman Urdu: URDF robot ki banaawat aur harkat ko bayan karta hai.
- Urdu Script: یو آر ڈی ایف روبوٹ کی بناوٹ اور حرکت کو بیان کرتا ہے۔
- English: SDF sim world aur physics ko muayyan karta hai.
- Roman Urdu: SDF sim world aur physics ko muayyan karta hai.
- Urdu Script: ایس ڈی ایف سم ورلڈ اور فزکس کو متعین کرتا ہے۔

