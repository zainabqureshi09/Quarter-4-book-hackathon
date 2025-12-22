---
id: ros2-architecture
title: "ROS 2 Architecture"
description: "Graph, middleware, QoS, executors, and composition for humanoid robotics."
sidebar_position: 1
---

# ROS 2 Architecture

## Beginner

ROS 2 is a communication framework that lets different robot parts exchange messages safely and efficiently. The system is a graph of nodes connected by topics, services, and actions. Nodes run separately, so one failure does not crash the whole robot. This modular design makes humanoid robots reliable and easier to scale.

## Intermediate

ROS 2 uses DDS as a transport layer to deliver messages with configurable Quality of Service. The graph consists of executable nodes that publish or subscribe to topics for streaming data, and services/actions for request–reply and long-running tasks. Executors schedule callbacks, timers, and subscriptions deterministically when configured with real-time kernels. Composition loads multiple components in a single process to reduce latency and memory overhead.

## Advanced

For humanoid control loops, deterministic latency is critical. Configure QoS for sensor streams as best-effort and control commands as reliable with transient local durability for late joiners. Use intra-process communication and shared memory to reduce serialization overhead. Place balance and locomotion callbacks in a priority executor on a PREEMPT_RT kernel. Profile end-to-end latency from sensor driver to controller output and enforce watchdogs to guarantee fail-safe behavior.

## ROS 2 Code Examples

```python
# file: qos_demo_publisher.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

class QosPub(Node):
    def __init__(self):
        super().__init__('qos_pub')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(String, 'control_cmd', qos)
        self.timer = self.create_timer(0.01, self.tick)
        self.i = 0

    def tick(self):
        msg = String()
        msg.data = str(self.i)
        self.pub.publish(msg)
        self.i += 1

def main():
    rclpy.init()
    node = QosPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# file: qos_demo_subscriber.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

class QosSub(Node):
    def __init__(self):
        super().__init__('qos_sub')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sub = self.create_subscription(String, 'control_cmd', self.cb, qos)

    def cb(self, msg):
        pass

def main():
    rclpy.init()
    node = QosSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid-Specific Explanations

- Humanoid balance controllers require reliable command channels and fast sensor streams.
- Transient local durability allows late-joining controllers to receive last known commands when restarting.
- Intra-process communication reduces copying between perception and control nodes co-located in the same process.

## Exercises

- Configure two topics with different QoS profiles and measure message loss.
- Run callbacks at 1 kHz using a timer and record jitter.
- Compare latency with composition versus separate processes.

## Assessments

- Multiple Choice: Which QoS durability ensures late joiners receive the last message?
- Short Answer: Why is intra-process communication beneficial for humanoid loops?
- Scenario: The robot stumbles when a perception node restarts. Which QoS change helps?

## RAG Summary Chunks

- Chunk 1: ROS 2 uses DDS transport with configurable QoS for reliability, durability, and performance.
- Chunk 2: Executors schedule node callbacks; composition reduces latency via shared process.
- Chunk 3: Humanoid control requires deterministic end-to-end latency and fail-safes.

## Urdu Translation

- English: ROS 2 robot ke hisson ko ba-vasila rabt deta hai.
- Roman Urdu: ROS 2 robot ke hisson ko bawasila rabt deta hai.
- Urdu Script: ROS 2 روبوٹ کے حصوں کو باوسیلہ ربط دیتا ہے۔
- English: QoS se paighamat ki bharosemandi aur taakhir control hoti hai.
- Roman Urdu: QoS se paighamat ki bharosemandi aur takhir control hoti hai.
- Urdu Script: QoS سے پیغامات کی بھروسہ مندی اور تاخیر کنٹرول ہوتی ہے۔

