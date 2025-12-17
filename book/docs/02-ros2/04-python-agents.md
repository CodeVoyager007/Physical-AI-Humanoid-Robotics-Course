---
title: "Python Agents: Writing Controllers with `rclpy`"
sidebar_label: "Python Agents"
---

# Python Agents: Writing Controllers with `rclpy`

Python is an incredibly popular language in robotics due to its readability, extensive libraries, and rapid prototyping capabilities. `rclpy` is the official Python client library for ROS 2, providing a full-featured API to interact with the ROS graph.

In this chapter, we will delve into writing functional ROS 2 nodes in Python, specifically focusing on how to create simple controllers or "agents" that can perceive (via subscriptions) and act (via publications) within the ROS ecosystem.

## Building Blocks of an `rclpy` Node

Every `rclpy` node typically involves:
1.  **Importing `rclpy` and `Node`**: The fundamental components.
2.  **Importing Message Types**: To handle data for topics, services, or actions.
3.  **Creating a Class that Inherits from `Node`**: Encapsulates your node's logic.
4.  **`__init__` Method**:
    -   Calls `super().__init__('node_name')` to initialize the base Node class.
    -   Creates publishers, subscribers, services, clients, or action clients/servers.
    -   Initializes timers for periodic tasks.
5.  **Callback Functions**: Methods that are executed when new messages arrive on subscribed topics, service requests are received, or timer events occur.
6.  **`main` Function**:
    -   Calls `rclpy.init(args=args)` to initialize the ROS context.
    -   Instantiates your `Node` class.
    -   Calls `rclpy.spin(node)` to keep the node alive and processing events.
    -   Calls `node.destroy_node()` and `rclpy.shutdown()` upon exit.

## Example: A Simple Differential Drive Controller

Let's imagine a robot that needs to move forward until it "sees" something, then stop. We can simulate this with a controller node that publishes `Twist` messages (movement commands) and subscribes to a simulated "obstacle detected" message (a simple boolean).

### `robot_controller.py`

This node will publish `Twist` messages to control the robot's linear and angular velocity. It will stop if it receives a `True` message on the `/obstacle_detected` topic.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool # For simulating obstacle detection

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        # Publisher for movement commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscriber for obstacle detection
        self.subscriber_ = self.create_subscription(
            Bool,
            '/obstacle_detected',
            self.obstacle_callback,
            10)
        self.subscriber_ # prevent unused variable warning
        
        self.is_moving = True
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Robot Controller node started. Moving forward...")

    def obstacle_callback(self, msg):
        if msg.data is True and self.is_moving:
            self.get_logger().info("Obstacle detected! Stopping robot.")
            self.stop_robot()
        elif msg.data is False and not self.is_moving:
            self.get_logger().info("Path clear! Resuming movement.")
            self.start_robot()

    def timer_callback(self):
        if self.is_moving:
            msg = Twist()
            msg.linear.x = 0.2 # Move forward at 0.2 m/s
            msg.angular.z = 0.0 # Don't turn
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Publishing: Linear X: {msg.linear.x}')

    def stop_robot(self):
        if self.is_moving:
            stop_msg = Twist() # All zeros
            self.publisher_.publish(stop_msg)
            self.is_moving = False

    def start_robot(self):
        if not self.is_moving:
            self.is_moving = True

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### `obstacle_sensor_simulator.py`

This node simulates a sensor that detects obstacles. It will publish `True` for a few seconds, then `False`.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class ObstacleSensorSimulator(Node):

    def __init__(self):
        super().__init__('obstacle_sensor_simulator')
        self.publisher_ = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Publish every second
        self.obstacle_present = False
        self.count = 0
        self.get_logger().info("Obstacle Sensor Simulator started.")

    def timer_callback(self):
        msg = Bool()
        if self.count < 5: # Simulate obstacle for 5 seconds
            msg.data = True
            self.obstacle_present = True
        else:
            msg.data = False
            self.obstacle_present = False
            
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing obstacle_detected: {msg.data}')
        self.count += 1
        
        # Reset after a while to make it repeat
        if self.count > 10: 
            self.count = 0

def main(args=None):
    rclpy.init(args=args)
    sensor_simulator = ObstacleSensorSimulator()
    rclpy.spin(sensor_simulator)
    sensor_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Agents

To run these two nodes, you would typically open two separate terminals in your ROS 2 workspace (after sourcing your setup file) and run each node:

**Terminal 1 (Robot Controller):**
```bash
python3 robot_controller.py
```

**Terminal 2 (Obstacle Sensor Simulator):**
```bash
python3 obstacle_sensor_simulator.py
```

You would observe the `robot_controller` starting to move, stopping when the `obstacle_sensor_simulator` publishes `True`, and resuming when it publishes `False`. This demonstrates the power of `rclpy` to build reactive and modular robotic behaviors.
