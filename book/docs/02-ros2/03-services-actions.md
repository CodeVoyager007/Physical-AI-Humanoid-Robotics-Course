---
title: "ROS 2 Services and Actions: Client/Server Patterns"
sidebar_label: "Services & Actions"
---

# ROS 2 Services and Actions: Client/Server Patterns

While Topics are excellent for continuous, asynchronous data streams, robotics often requires direct, request-response communication. ROS 2 provides two main client/server patterns for this: **Services** for simple, synchronous calls, and **Actions** for long-running, goal-oriented tasks.

## Services: Synchronous Request/Response

A ROS 2 Service is a simple remote procedure call (RPC) mechanism. A **client** sends a request to a **server**, and the server processes it and sends back a single response. This interaction is synchronous and blocking from the client's perspective: the client waits for the response before proceeding.

Services are ideal for:
-   Triggering an event (e.g., "take a picture").
-   Querying information (e.g., "get robot's current pose").
-   Changing a configuration (e.g., "set motor speed to X").

### Service Definition

Services are defined in `.srv` files. A service definition consists of a request part and a response part, separated by `---`.

Example: `example_interfaces/srv/AddTwoInts.srv`

```
int64 a
int64 b
---
int64 sum
```

### Creating a Service Server (Python)

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Import the service type

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client (Python)

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions: Goal-Oriented Feedback and Preemption

Actions are built on top of Services and Topics, designed for tasks that take a long time to execute, can be preempted (cancelled), and provide continuous feedback. Think of navigating a robot to a target location: it's a long-running process, you want to know its progress, and you might want to stop it midway.

An Action consists of:
-   **Goal**: The request to perform the task.
-   **Result**: The final outcome of the task.
-   **Feedback**: Intermediate updates on the progress of the task.

### Action Definition

Actions are defined in `.action` files, which combine a goal, result, and feedback message definition, each separated by `---`.

Example: `example_interfaces/action/Fibonacci.action`

```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

### Action Client/Server Interaction

The interaction with an Action server is more complex than with a Service, involving asynchronous communication for goals, results, and feedback.

**Client**:
1.  Sends a Goal.
2.  Receives continuous Feedback.
3.  Receives a final Result (or cancellation notification).

**Server**:
1.  Receives a Goal.
2.  Continuously sends Feedback.
3.  Sends a final Result (or handles preemption).

Actions are crucial for complex, multi-step robotic behaviors where monitoring progress and dynamic intervention are necessary.

Understanding when to use Topics, Services, and Actions is fundamental to designing robust and efficient ROS 2 applications. Each communication pattern serves a distinct purpose, and choosing the right one for the job is key to building sophisticated robotic systems.
