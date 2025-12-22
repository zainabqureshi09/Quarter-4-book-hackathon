---
id: rclpy-python-agents
title: "rclpy and Python Agents"
description: "Authoring Python nodes, parameters, timers, composition, and agent orchestration."
sidebar_position: 3
---

# rclpy and Python Agents

## Beginner

Python nodes let you write robot logic quickly. A node can publish messages, subscribe to topics, set timers, and manage parameters. Agents are coordinating nodes that supervise others and make decisions.

## Intermediate

Use `rclpy` to build nodes with timers and parameters for runtime tuning. Composition packs multiple components into a single process. Agents implement event loops that read sensors and schedule actions. Parameters allow live reconfiguration of gains and thresholds without restarting.

## Advanced

Design agents with finite state machines and guard conditions. Use multi-threaded or multi-process executors to isolate critical callbacks. Apply parameter events and dynamic reconfigure patterns. Persist agent state, record decisions, and enforce safety with watchdogs and heartbeats.

## ROS 2 Code Examples

```python
# file: agent_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Agent(Node):
    def __init__(self):
        super().__init__('agent')
        self.declare_parameter('threshold', 0.5)
        self.pub = self.create_publisher(String, 'agent_cmd', 10)
        self.sub = self.create_subscription(String, 'sensor_status', self.cb, 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.state = 'IDLE'

    def cb(self, msg):
        pass

    def tick(self):
        msg = String()
        msg.data = self.state
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = Agent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# file: parameter_watcher.py
import rclpy
from rclpy.node import Node

class ParamWatch(Node):
    def __init__(self):
        super().__init__('param_watch')
        self.add_on_set_parameters_callback(self.on_set)

    def on_set(self, params):
        return rclpy.parameter.SetParametersResult(successful=True)

def main():
    rclpy.init()
    node = ParamWatch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid-Specific Explanations

- Agents orchestrate perception and control while enforcing safety transitions.
- Parameters expose controller gains and state thresholds for tuning.
- Timers implement periodic tasks like heartbeat publishing and balance updates.

## Exercises

- Build an agent that transitions from IDLE to WALK when a topic reports stable support.
- Add a parameter that switches gait speed and verify changes live.
- Record agent commands into `rosbag2` and review timing.

## Assessments

- Multiple Choice: Which feature allows live tuning of controller gains?
- Short Answer: Why use a timer for heartbeat messages?
- Scenario: The agent must preempt walking if vision reports obstacle. Which design helps?

## RAG Summary Chunks

- Chunk 1: rclpy builds Python nodes with timers, parameters, and pub/sub.
- Chunk 2: Agents coordinate subsystems using state machines and guard conditions.
- Chunk 3: Parameters enable live tuning for humanoid controllers and safety.

## Urdu Translation

- English: Python agents nizam ko humahangi se chalate hain.
- Roman Urdu: Python agents nizam ko humahangi se chalate hain.
- Urdu Script: پائتھن ایجنٹس نظام کو ہم آہنگی سے چلاتے ہیں۔
- English: Parameters se robot ko baghair restart ke adjust kiya jata hai.
- Roman Urdu: Parameters se robot ko baghair restart ke adjust kiya jata hai.
- Urdu Script: پیرا میٹرز سے روبوٹ کو بغیر رِی اسٹارٹ کے ایڈجسٹ کیا جاتا ہے۔

