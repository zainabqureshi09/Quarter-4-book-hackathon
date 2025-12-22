---
id: physics-fundamentals
title: "Physics Simulation Fundamentals"
description: "Time step, solvers, contact, constraints, and stability for humanoids."
sidebar_position: 2
---

# Physics Simulation Fundamentals

## Beginner

Physics simulation imitates gravity, mass, collisions, and friction. It updates the world in small time steps and moves objects according to forces. Stable simulation keeps humanoid feet from sliding and prevents the robot from passing through the floor.

## Intermediate

Engines compute rigid-body dynamics and contact. Core parameters include time step, solver iterations, and max contacts. Small time steps increase accuracy but reduce speed. Too few solver iterations produce jitter at foot contact. Humanoids require precise contact modeling and tuned friction to stand and walk.

## Advanced

Use semi-implicit integration and constraint solvers with high iteration counts at foot contacts. Tune friction cone approximations and restitution to avoid bounce. Increase contact max substeps for impulses. Keep visual meshes separate from collision geometry to reduce computational load. Profile real-time factor and ensure deterministic stepping for control loops.

## Gazebo Configuration Example

```xml
<sdf version="1.9">
  <world name="humanoid_world">
    <gravity>0 0 -9.81</gravity>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>0.1</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
  </world>
</sdf>
```

## Humanoid Context

- Feet need high solver iterations and tuned friction to prevent slip.
- Hands benefit from soft contacts to improve grasp realism.
- Balance controllers require deterministic time steps and low jitter.

## Exercises

- Change time step and measure effect on joint stability.
- Increase solver iterations and observe contact jitter reduction.
- Compare collision mesh complexity vs real-time factor.

## Assessments

- Multiple Choice: Which parameter reduces contact jitter at humanoid feet?
- Short Answer: Why separate visual and collision meshes?
- Scenario: Real-time factor drops below 1.0. What adjustments help?

## RAG Summary Chunks

- Chunk 1: Physics engines simulate gravity, mass, friction, and collisions with time steps.
- Chunk 2: Solver iterations and constraints drive contact stability for humanoids.
- Chunk 3: Deterministic stepping and tuned friction improve balance and walking.

## Urdu Translation

- English: Physics simulation me jism quwwat aur takrao se harkat karta hai.
- Roman Urdu: Physics simulation me jism quwwat aur takrao se harkat karta hai.
- Urdu Script: فزکس سمیولیشن میں جسم قوت اور ٹکراؤ سے حرکت کرتا ہے۔
- English: Chhota time step se accuracy barhti hai magar raftaar kam hoti hai.
- Roman Urdu: Chhota time step se accuracy barhti hai magar raftaar kam hoti hai.
- Urdu Script: چھوٹا ٹائم اسٹیپ سے درستگی بڑھتی ہے مگر رفتار کم ہوتی ہے۔

