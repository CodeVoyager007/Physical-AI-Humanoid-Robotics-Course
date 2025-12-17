---
title: "ROS 2 Architecture: Nodes, Contexts, and DDS"
sidebar_label: "Architecture"
---

# ROS 2 Architecture: Nodes, Contexts, and DDS

Welcome to the core of modern robotics software. The Robot Operating System (ROS) is not an operating system in the traditional sense (like Windows or Linux). Instead, it is a **middleware**—a framework and set of tools that provide services you'd expect from an OS, such as hardware abstraction, low-level device control, message passing, and package management.

ROS 2 is a complete redesign of the original ROS, built to address the needs of commercial and mission-critical applications.

## The Fundamental Unit: The Node

Everything in ROS is a **Node**. A node is a single, self-contained, executable process responsible for one specific task. This modular design is the cornerstone of ROS's power and flexibility.

Consider a simple mobile robot. You might have several nodes:
-   A `camera_driver` node that reads raw data from a physical camera.
-   An `image_processor` node that detects obstacles in the camera image.
-   A `lidar_driver` node that gets data from a laser scanner.
-   A `path_planner` node that uses obstacle data to compute a safe path.
-   A `motor_controller` node that sends commands to the wheels.

Each node is a separate program. They can be written in different languages (C++ and Python are the most common), run on different computers, and be started and stopped independently. This loose coupling makes the system robust, scalable, and easy to debug.

:::tip The Single Responsibility Principle
A well-designed ROS system follows the single responsibility principle. Each node should do one thing and do it well. This makes nodes reusable and the entire system easier to understand.
:::

## The Communication Backbone: DDS

How do all these independent nodes talk to each other? ROS 1 had its own custom communication system (TCPROS). ROS 2 made a pivotal design decision: to build on top of an existing, industry-standard protocol called **DDS (Data Distribution Service)**.

DDS is a publish-subscribe middleware standard maintained by the Object Management Group (OMG), the same consortium that manages standards like UML and CORBA. It's used in mission-critical systems like air traffic control, high-frequency trading, and industrial automation.

### Why DDS?

-   **Discovery**: DDS provides a built-in, automatic discovery mechanism. When you start a new node, it automatically announces its presence and discovers all other nodes on the network without any central server.
-   **Quality of Service (QoS)**: This is the killer feature. DDS allows you to specify *how* data should be delivered. Should it be reliable (like TCP) or best-effort (like UDP)? Should it keep a history of the last 10 messages? Should it be "transient local" (so a new subscriber instantly gets the last sent message)? These QoS policies are essential for building robust robotic systems.
-   **Performance**: DDS implementations are highly optimized for low-latency, high-throughput data transfer.
-   **Interoperability**: By using a standard, different vendors can provide their own high-performance DDS implementations (e.g., Fast DDS, Cyclone DDS), and they can all interoperate.

:::danger Abstraction
While ROS 2 uses DDS under the hood, you rarely interact with it directly. ROS 2 provides a client library (`rclcpp` for C++, `rclpy` for Python) that gives you a clean, user-friendly API for creating nodes, publishers, and subscribers. You simply set the QoS policies you need, and ROS handles the DDS implementation details for you.
:::

## The Application Entry Point: The Context

A **Context** (`rclcpp::Context` or `rclpy.context.Context`) is an object that represents the initialization and shutdown of ROS communications in your application.

When your program starts, you initialize the ROS context. This action sets up the underlying DDS participant and begins the discovery process. All nodes you create within your application will belong to this context.

```python
import rclpy

def main(args=None):
    # 1. Initialize the ROS Context
    rclpy.init(args=args)

    # ... create nodes here ...

    # 2. Shutdown the ROS Context
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

In summary, the ROS 2 architecture is a distributed system of single-purpose **Nodes** that communicate over a standardized, high-performance **DDS** backbone. Your application manages its participation in this system through a **Context**. This modular, decoupled architecture is what makes ROS 2 the ideal framework for building complex, scalable, and robust robotic systems.
