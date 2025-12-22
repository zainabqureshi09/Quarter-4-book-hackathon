---
id: sensor-simulation
title: "Sensor Simulation: LiDAR, Depth Camera, IMU"
description: "Configuring simulated sensors and ROS 2 bridges for humanoids."
sidebar_position: 5
---

# Sensor Simulation: LiDAR, Depth Camera, IMU

## Beginner

Simulated sensors mimic real devices. LiDAR measures distances, depth cameras provide 3D images, and IMUs report orientation and acceleration. These help humanoids see and balance in the virtual world.

## Intermediate

Attach sensor plugins to models and publish ROS 2 topics. Tune update rates and resolution. Use TF frames to align sensors with robot links. Test perception nodes against simulated topics.

## Advanced

Optimize sensor rates to balance fidelity and performance. Synchronize sensors with message filters. Calibrate noise models for robust algorithms. Use headless rendering when possible and ensure frame consistency.

## Gazebo SDF Configuration Examples

```xml
<sensor name="lidar" type="ray">
  <update_rate>15</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>1024</samples>
        <min_angle>-1.57</min_angle>
        <max_angle>1.57</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>20.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="gazebo_ros_laser" filename="libgazebo_ros_laser.so">
    <topicName>/scan</topicName>
    <frameName>head_link</frameName>
  </plugin>
</sensor>
```

```xml
<sensor name="depth_cam" type="depth">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.0</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <plugin name="gazebo_ros_depth_camera" filename="libgazebo_ros_depth_camera.so">
    <imageTopicName>/camera/color/image_raw</imageTopicName>
    <depthImageTopicName>/camera/depth/image_raw</depthImageTopicName>
    <cameraInfoTopicName>/camera/color/camera_info</cameraInfoTopicName>
    <frameName>head_link</frameName>
  </plugin>
</sensor>
```

```xml
<sensor name="imu" type="imu">
  <update_rate>200</update_rate>
  <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
    <topicName>/imu/data</topicName>
    <frameName>base_link</frameName>
  </plugin>
</sensor>
```

## Humanoid Context

- Head-mounted depth camera supports grasp and obstacle detection.
- IMU provides orientation for balance and fall detection.
- LiDAR assists in mapping rooms and navigation for humanoids.

## Exercises

- Attach sensors to a humanoid head link and verify topics.
- Subscribe to IMU and compute roll, pitch, yaw.
- Synchronize depth and RGB streams and test a perception node.

## Assessments

- Multiple Choice: Which sensor reports orientation and acceleration?
- Short Answer: Why synchronize RGB and depth streams?
- Scenario: Perception lags. Which sensor parameters should be tuned?

## RAG Summary Chunks

- Chunk 1: Simulated LiDAR, depth, and IMU publish ROS 2 topics via plugins.
- Chunk 2: Frame alignment and update rates are critical for humanoid tasks.
- Chunk 3: Synchronization and noise models improve robustness.



