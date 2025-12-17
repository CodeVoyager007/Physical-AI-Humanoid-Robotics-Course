---
title: "ROS 2 Communication: Topics (Pub/Sub) and Messages"
sidebar_label: "Communication"
---

# ROS 2 Communication: Topics (Pub/Sub) and Messages

The heart of any distributed robotic system is how its components communicate. In ROS 2, the primary mechanism for asynchronous, many-to-many communication is through **Topics**, using a **Publish-Subscribe** model.

## The Publish-Subscribe Model

Imagine a busy newsroom. Instead of reporters directly sending articles to every single reader, they publish their stories to different news channels (topics). Readers subscribe to the channels they are interested in.

In ROS 2:
-   **Publishers (Reporters)**: Nodes that create and send data.
-   **Subscribers (Readers)**: Nodes that receive and process data.
-   **Topics (News Channels)**: Named buses over which data flows.

A publisher sends data to a specific topic, and any node that has subscribed to that topic receives the data. Crucially, publishers and subscribers don't know about each other directly. They only know about the topic. This decoupling is what makes ROS 2 systems flexible and scalable.

:::tip Anonymity and Decoupling
Publishers and subscribers are completely decoupled. They can join or leave the network at any time without affecting the others. This makes it easy to swap out components, debug, or add new functionality.
:::

## Messages: The Language of ROS 2

When data is sent over a topic, it must conform to a predefined structure called a **Message**. ROS 2 messages are strongly typed, ensuring that all publishers and subscribers agree on the format of the data being exchanged.

Message types are defined in `.msg` files, which are compiled into source code in various languages (e.g., Python classes, C++ structs) that your nodes can use.

### Common Message Types:

-   `std_msgs/msg/String`: A simple string.
-   `std_msgs/msg/Int32`: A 32-bit integer.
-   `geometry_msgs/msg/Twist`: Represents linear and angular velocity, commonly used to command robot movement.
-   `sensor_msgs/msg/LaserScan`: Data from a 2D laser scanner.
-   `sensor_msgs/msg/Image`: Image data from a camera.

### Example: `geometry_msgs/msg/Twist`

```python
# A typical Twist message
# Linear velocity in x, y, z axes
linear:
  x: 0.5  # move forward at 0.5 m/s
  y: 0.0
  z: 0.0
# Angular velocity in x, y, z axes (for rotation around axes)
angular:
  x: 0.0
  y: 0.0
  z: 0.2  # turn counter-clockwise at 0.2 rad/s
```

## Creating a Publisher (Python)

Let's look at a simple Python example of a node that publishes `Twist` messages to a topic named `/cmd_vel` (a common topic for robot velocity commands).

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5 # Move forward
        msg.angular.z = 0.0 # Don't turn
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "Linear X: {msg.linear.x}, Angular Z: {msg.angular.z}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher) # Keep the node alive until Ctrl+C
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Subscriber (Python)

And a corresponding subscriber that prints the received `Twist` messages.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "Linear X: {msg.linear.x}, Angular Z: {msg.angular.z}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This publish-subscribe model, combined with strongly typed messages, forms the fundamental communication pattern in ROS 2, allowing for robust and flexible distributed robotic applications.
