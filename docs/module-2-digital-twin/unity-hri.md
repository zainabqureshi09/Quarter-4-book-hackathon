---
id: unity-hri
title: "Unity for High-Fidelity HRI"
description: "Unity Robotics Hub, ROS–TCP Bridge, and interactive HRI scenes."
sidebar_position: 6
---

# Unity for High-Fidelity HRI

## Beginner

Unity renders realistic scenes and characters. With the ROS–TCP Bridge, Unity connects to ROS 2 nodes to exchange messages. This creates interactive environments to test humanoid behavior with humans.

## Intermediate

Install Unity Robotics Hub packages and add ROS–TCP Connector and Endpoint. Configure IP and port to match the ROS 2 bridge. Publish camera images and subscribe to motion commands. Build simple HRI scenes that include human avatars and props.

## Advanced

Use HDRP for realism and physics materials for contact. Stream multiple sensors from Unity to ROS 2. Implement interaction scripts and synchronize TF frames. Integrate domain randomization for robust perception. Profile frame time and networking latency.

## Unity Configuration Example

```csharp
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using RosMessageTypes.Std;

public class UnityRosPublisher : MonoBehaviour
{
    ROSConnection ros;
    string topic = "/unity_status";
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topic);
    }
    void Update()
    {
        var msg = new StringMsg("ready");
        ros.Publish(topic, msg);
    }
}
```

## Humanoid Context

- Human avatars test social distances and gesture responses.
- Photorealistic rendering improves perception training.
- Unity enables rapid prototyping of complex HRI scenarios.

## Exercises

- Configure ROS–TCP and publish a status message from Unity.
- Subscribe in ROS 2 and log the Unity message.
- Create an HRI scene with gestures and verify message flow.

## Assessments

- Multiple Choice: Which Unity package links to ROS messages?
- Short Answer: Why use photorealistic rendering for perception training?
- Scenario: Unity messages arrive late. Which settings should be reviewed?

## RAG Summary Chunks

- Chunk 1: Unity Robotics Hub bridges Unity and ROS 2 via ROS–TCP.
- Chunk 2: High-fidelity rendering supports HRI and perception training.
- Chunk 3: Networking configuration and profiling ensure low-latency messaging.

## Urdu Translation

- English: Unity ROS se rabt banata hai taake HRI scenes chal sakein.
- Roman Urdu: Unity ROS se rabt banata hai taake HRI scenes chal sakein.
- Urdu Script: یونٹی آر او ایس سے ربط بناتا ہے تاکہ ایچ آر آئی مناظر چل سکیں۔
- English: Photorealistic scenes se perception ki training behtar hoti hai.
- Roman Urdu: Photorealistic scenes se perception ki training behtar hoti hai.
- Urdu Script: فوٹو ریالسٹک مناظر سے پرسیپشن کی تربیت بہتر ہوتی ہے۔

