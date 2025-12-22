---
id: ros2-communication
title: "Nodes, Topics, Services, Actions"
description: "Communication patterns for humanoid robotics in ROS 2."
sidebar_position: 2
---

# Nodes, Topics, Services, Actions

## Beginner

Nodes are independent programs. Topics stream data between nodes. Services answer direct requests. Actions manage long-running tasks with feedback. This separation lets humanoid robots coordinate vision, balance, and locomotion safely.

## Intermediate

Publishers and subscribers connect through topics for asynchronous messaging. Services use request–response for atomic operations. Actions handle goals, feedback, and cancellation. Use namespaces and remapping to organize large humanoid systems and avoid name collisions across subsystems.

## Advanced

Use message filters and synchronized subscriptions for time-aligned sensor fusion. Choose QoS best-effort for high-rate sensors and reliable for control. Implement idempotent services and cancellable actions for robust behavior under failures. Integrate actions with planners for preemption.

## ROS 2 Code Examples

```python
# file: basic_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BasicPub(Node):
    def __init__(self):
        super().__init__('basic_pub')
        self.pub = self.create_publisher(String, 'status', 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.i = 0

    def tick(self):
        msg = String()
        msg.data = str(self.i)
        self.pub.publish(msg)
        self.i += 1

def main():
    rclpy.init()
    node = BasicPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# file: basic_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BasicSub(Node):
    def __init__(self):
        super().__init__('basic_sub')
        self.sub = self.create_subscription(String, 'status', self.cb, 10)

    def cb(self, msg):
        pass

def main():
    rclpy.init()
    node = BasicSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# file: service_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddServer(Node):
    def __init__(self):
        super().__init__('add_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.cb)

    def cb(self, request, response):
        response.sum = request.a + request.b
        return response

def main():
    rclpy.init()
    node = AddServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# file: service_client.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            pass
        req = AddTwoInts.Request()
        req.a = 2
        req.b = 3
        self.future = self.cli.call_async(req)

def main():
    rclpy.init()
    node = AddClient()
    rclpy.spin_until_future_complete(node, node.future)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# file: action_server.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class FibServer(Node):
    def __init__(self):
        super().__init__('fib_server')
        self.server = ActionServer(self, Fibonacci, 'fib', self.execute)

    async def execute(self, goal_handle):
        feedback = Fibonacci.Feedback()
        result = Fibonacci.Result()
        seq = [0, 1]
        for i in range(1, goal_handle.request.order):
            seq.append(seq[i] + seq[i-1])
            feedback.partial_sequence = seq
            goal_handle.publish_feedback(feedback)
        result.sequence = seq
        goal_handle.succeed()
        return result

def main():
    rclpy.init()
    node = FibServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# file: action_client.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci

class FibClient(Node):
    def __init__(self):
        super().__init__('fib_client')
        self.client = ActionClient(self, Fibonacci, 'fib')
        self.client.wait_for_server()
        goal = Fibonacci.Goal()
        goal.order = 5
        self.future = self.client.send_goal_async(goal)

def main():
    rclpy.init()
    node = FibClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid-Specific Explanations

- Topics stream joint states and sensor frames at high rates.
- Services handle discrete commands like mode switches.
- Actions execute trajectories with feedback and preemption support.

## Exercises

- Create a publisher for joint positions and a subscriber that detects out-of-range values.
- Implement a service that resets controllers.
- Implement an action that executes a timed trajectory with cancellation.

## Assessments

- Multiple Choice: Which pattern suits long-running motion with feedback?
- Short Answer: Why prefer best-effort for high-rate sensors?
- Scenario: The robot must stop mid-trajectory. Which interface supports this?

## RAG Summary Chunks

- Chunk 1: Topics are asynchronous streams; services are request–response; actions manage goals with feedback.
- Chunk 2: Humanoids use actions for trajectories and topics for joint states.
- Chunk 3: QoS and synchronization ensure reliable control and perception.

## Urdu Translation

- English: Topics musalsal data rawana karte hain.
- Roman Urdu: Topics musalsal data rawana karte hain.
- Urdu Script: ٹاپکس مسلسل ڈیٹا روانہ کرتے ہیں۔
- English: Actions lambi muddat ke goals aur feedback sambhalte hain.
- Roman Urdu: Actions lambi muddat ke goals aur feedback sambhalte hain.
- Urdu Script: ایکشنز لمبی مدت کے اہداف اور فیڈ بیک سنبھالتے ہیں۔

