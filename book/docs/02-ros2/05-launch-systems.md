---
title: "ROS 2 Launch Systems: Managing Complex Robot Starts"
sidebar_label: "Launch Systems"
---

# ROS 2 Launch Systems: Managing Complex Robot Starts

As your robot's software grows, you'll quickly accumulate many ROS 2 nodes, potentially distributed across multiple machines. Manually starting each node in a separate terminal is tedious, error-prone, and unsustainable. This is where **ROS 2 Launch Systems** come in.

ROS 2 Launch is a powerful tool designed to:
-   Start multiple nodes simultaneously.
-   Remap topics, services, and actions.
-   Set parameters for nodes.
-   Include other launch files, creating modular startup configurations.
-   Execute external commands or programs.
-   Provide conditional execution (e.g., only launch a sensor node if the hardware is present).

Launch files are typically written in Python, offering maximum flexibility and programmatic control over the startup process.

## Basic Structure of a Launch File

A ROS 2 Python launch file is essentially a Python script that defines a `generate_launch_description` function. This function returns a `LaunchDescription` object, which is a collection of actions that `ros2 launch` will execute.

```python
# my_robot_bringup/launch/my_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start a simple Python talker node
        Node(
            package='py_pubsub', # The package containing the node
            executable='talker', # The executable name of the node
            name='my_talker_node', # The desired name for this specific node instance
            namespace='robot1', # Optional: put this node in a namespace
            output='screen', # Show log messages to the console
            emulate_tty=True, # Important for Python nodes to show buffered output immediately
            parameters=[ # Parameters for the node (e.g., from YAML file or dict)
                {'message_frequency': 10.0}
            ]
        ),
        # Start a simple C++ listener node
        Node(
            package='cpp_pubsub',
            executable='listener',
            name='my_listener_node',
            namespace='robot1',
            output='screen',
            emulate_tty=True,
        )
    ])
```

## Key Launch Actions

The `launch` and `launch_ros` packages provide a rich set of actions you can use:

-   **`Node`**: The most common action, used to start a ROS 2 executable (node).
-   **`ExecuteProcess`**: Runs any arbitrary shell command. Useful for starting non-ROS programs or scripts.
-   **`IncludeLaunchDescription`**: Includes another launch file. This is vital for modularity; you can have separate launch files for different robot components (e.g., `sensors.launch.py`, `navigation.launch.py`) and combine them in a top-level `robot_bringup.launch.py`.
-   **`GroupAction`**: Groups multiple actions together, often used with `PushRosNamespace` to apply a namespace to a set of nodes.
-   **`PushRosNamespace`**: Adds a namespace to all subsequent nodes or actions within its scope.
-   **`SetParameter` / `SetParametersFromFile`**: Sets parameters for nodes.

## Example: Using Namespaces and Including Other Launch Files

Namespaces are crucial for multi-robot systems or for running multiple instances of the same robot software without conflicts.

```python
# my_robot_bringup/launch/multi_robot_demo.launch.py
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory # Helper to find package resources
import os

def generate_launch_description():
    # Path to a generic robot's core launch file (e.g., assuming it exists)
    # Let's say 'my_base_robot_package' has 'base_robot.launch.py'
    base_robot_launch_dir = os.path.join(
        get_package_share_directory('my_base_robot_package'),
        'launch'
    )
    base_robot_launch_file = os.path.join(base_robot_launch_dir, 'base_robot.launch.py')

    return LaunchDescription([
        # Launch robot 1 in its own namespace
        GroupAction(actions=[
            PushRosNamespace('robot1'), # Apply 'robot1' namespace to everything below
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_robot_launch_file),
                launch_arguments={'robot_id': 'R1'}.items() # Pass arguments if base_robot.launch.py accepts them
            )
        ]),

        # Launch robot 2 in its own namespace
        GroupAction(actions=[
            PushRosNamespace('robot2'), # Apply 'robot2' namespace to everything below
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_robot_launch_file),
                launch_arguments={'robot_id': 'R2'}.items()
            )
        ])
    ])
```

The ROS 2 Launch System is an indispensable tool for managing the complexity of real-world robotic applications. By leveraging Python's flexibility, it allows you to create robust, reproducible, and scalable startup configurations for even the most intricate robot deployments.
