---
sidebar_position: 1
id: digital-twin-intro
title: "Module 2: Digital Twin – Gazebo & Unity"
description: "Simulating humanoid physics and environments using Gazebo and Unity."
---

# Module 2: Digital Twin – Gazebo & Unity

## 1. Learning Objectives
By the end of this chapter, you will be able to:
- **Define** what a Digital Twin is and its role in robotics development.
- **Compare** Gazebo (physics-focused) and Unity (visual/interaction-focused).
- **Import** a URDF (Unified Robot Description Format) model into a simulation.
- **Simulate** basic physics interactions (gravity, collision).

---

## 2. Core Explanation (Intermediate)

A **Digital Twin** is a virtual replica of a physical robot and its environment. Before building expensive hardware, roboticists use simulators to test code, train AI, and verify safety.

**Gazebo**:
- The standard simulator for ROS 2.
- Uses high-fidelity physics engines (ODE, Bullet, DART) to calculate forces, friction, and joint dynamics accurately.
- Best for: Testing control algorithms, sensor drivers (LiDAR, Cameras), and navigation.

**Unity**:
- A game engine increasingly used for robotics via the **Unity Robotics Hub**.
- Provides photorealistic rendering and complex physics.
- Best for: Human-Robot Interaction (HRI), Reinforcement Learning (training AI in visually rich worlds), and VR teleoperation.

**URDF (Unified Robot Description Format)**:
- An XML file format that describes what the robot looks like and how it moves (links and joints). Both Gazebo and Unity use this to build the virtual robot.

---

## 3. Beginner Simplification

Think of a **Video Game**:

- When you play a racing game, you can crash the car without paying for repairs.
- **Gazebo/Unity** are like that video game for robots.
- If you write code that makes the robot fall over, it falls over on the screen, not in the lab. This saves money and time.
- **URDF** is the "Character Profile" that tells the game what the robot looks like (tall, short, wheels, legs).

---

## 4. Advanced Deep-Dive

### Physics Engines & Contact Dynamics
Simulating a walking humanoid is hard because of **contact dynamics**. When a foot hits the ground, the physics engine must solve complex differential equations to prevent the foot from passing through the floor while applying the correct reaction force.
- **Hard Contacts**: Idealized, instant collision. Can cause "jittering" in simulation.
- **Soft Contacts**: Allow slight deformation, more realistic for rubber feet.

### Sim-to-Real Gap
A major challenge is the **Sim-to-Real Gap**. An AI trained in Unity might fail in the real world because the simulation wasn't perfect (e.g., friction was slightly off, or lighting was different).
- **Domain Randomization**: A technique where we randomize colors, friction, and mass in the simulation so the AI learns to be robust to changes, helping it transfer to the real world.

---

## 5. Code Examples

### URDF Snippet (XML)
A simple "leg" definition.

```xml
<robot name="simple_leg">
  <!-- The Hip Link -->
  <link name="hip">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- The Leg Link -->
  <link name="upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- The Joint connecting them -->
  <joint name="hip_joint" type="revolute">
    <parent link="hip"/>
    <child link="upper_leg"/>
    <origin xyz="0 0 -0.05"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.5" upper="1.5" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

### Spawning in Gazebo (Command Line)
Assuming you have the `gazebo_ros` package installed.

```bash
ros2 launch gazebo_ros gazebo.launch.py
ros2 run gazebo_ros spawn_entity.py -file simple_leg.urdf -entity my_leg
```

---

## 6. Real-World Humanoid Robotics Context

Companies like **Boston Dynamics** and **Agility Robotics** spend thousands of hours in simulation before turning on the real robot.
- **Safety**: A 150lb robot falling can break itself or hurt engineers.
- **Training**: AI models (like walking policies) need millions of trials to learn. You can run 1,000 simulations in parallel in the cloud (faster than real time), which is impossible with physical hardware.

---

## 7. Exercises

1.  **Setup**: Install Gazebo and the ROS 2 Gazebo packages.
2.  **Create**: Write a URDF file for a simple pendulum (a fixed base and a moving arm).
3.  **Simulate**: Spawn your pendulum in Gazebo.
4.  **Observe**: What happens if you don't set a `<limit>` on the joint? Does it spin forever?

---

## 8. Assessment Questions

1.  **True/False**: Unity is primarily used for its high-fidelity physics calculations compared to Gazebo. (False - Unity is visual/interaction, Gazebo is often preferred for pure rigid body physics integration with ROS, though Unity is catching up).
2.  **Short Answer**: What is the "Sim-to-Real Gap"?
3.  **Coding**: In a URDF, what tag is used to define the physical boundaries for collision detection? (`<collision>`)

---

## 9. RAG-Friendly Summaries

**Chunk 1: Digital Twin Definition**
A Digital Twin is a virtual simulation of a physical system. In robotics, it allows for safe testing, rapid prototyping, and AI training (Reinforcement Learning) without the risks associated with hardware.

**Chunk 2: URDF**
URDF (Unified Robot Description Format) is the standard XML format for representing robot models in ROS. It defines links (parts), joints (movements), visual meshes, and collision geometries.

**Chunk 3: Sim-to-Real Transfer**
The Sim-to-Real gap refers to the discrepancy between simulation and reality. Techniques like Domain Randomization are used to train AI models that can generalize from the simulation to the physical world.

---
