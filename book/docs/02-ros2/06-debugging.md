---
title: "ROS 2 Debugging: Using RQT and Rosbags"
sidebar_label: "Debugging"
---

# ROS 2 Debugging: Using RQT and Rosbags

Debugging distributed, real-time systems like those built with ROS 2 can be challenging. Fortunately, ROS 2 provides powerful tools that help visualize, inspect, and record data, making the debugging process significantly more manageable. This chapter introduces two indispensable tools: **RQT** for real-time introspection and **Rosbags** for offline data analysis.

## RQT: The ROS 2 GUI Toolkit

RQT is a suite of graphical user interface (GUI) tools that allow you to introspect and interact with a running ROS 2 system. It's a modular framework, meaning you can open various RQT plugins (like plotters, topic monitors, configuration tools) within a single RQT window.

To launch RQT:
```bash
ros2 run rqt_gui rqt_gui
```

### Essential RQT Plugins for Debugging:

1.  **`rqt_graph` (Node Graph)**:
    -   **Purpose**: Visualizes the ROS 2 computation graph, showing nodes and the topics, services, and actions connecting them. This is often the first tool to check if you suspect communication issues.
    -   **Usage**: From RQT, select `Plugins > Introspection > Node Graph`.
    -   **Debugging Insight**: Helps identify if nodes are starting correctly, if topics are being published and subscribed to, and if there are any unexpected connections.

2.  **`rqt_console` (Log Viewer)**:
    -   **Purpose**: Displays log messages (DEBUG, INFO, WARN, ERROR, FATAL) from all ROS 2 nodes in a central location. You can filter messages by node name, topic, severity, etc.
    -   **Usage**: From RQT, select `Plugins > Utilities > Console`.
    -   **Debugging Insight**: Essential for monitoring node behavior and quickly spotting errors or warnings as they occur.

3.  **`rqt_topic` (Topic Monitor)**:
    -   **Purpose**: Allows you to inspect the data flowing on any topic in real-time. You can view the message rate, bandwidth, and the actual content of messages.
    -   **Usage**: From RQT, select `Plugins > Topics > Topic Monitor`.
    -   **Debugging Insight**: Confirming that publishers are sending the correct data and subscribers are receiving it as expected. You can even publish messages manually to topics for testing.

4.  **`rqt_plot` (Data Plotter)**:
    -   **Purpose**: Plots numerical data from topics over time. This is invaluable for visualizing sensor readings, motor commands, or controller outputs.
    -   **Usage**: From RQT, select `Plugins > Visualization > Plot`.
    -   **Debugging Insight**: Quickly see trends, oscillations, or unexpected spikes in your robot's behavior.

5.  **`rqt_param` (Parameter Editor)**:
    -   **Purpose**: View and modify ROS 2 node parameters dynamically.
    -   **Usage**: From RQT, select `Plugins > Configuration > Dynamic Reconfigure`. (Note: This is an older plugin name, direct parameter interaction is often through `ros2 param` CLI tools).
    -   **Debugging Insight**: Adjusting controller gains or sensor thresholds on-the-fly without restarting nodes.

## Rosbags: Recording and Playback

Rosbags are files used to record data published on ROS 2 topics. This recorded data can then be played back later, allowing you to re-analyze robot behavior offline, develop algorithms without requiring the physical robot, and debug issues that are difficult to reproduce in real-time.

### Recording a Rosbag

To record all topics (or a subset of topics) to a file:
```bash
ros2 bag record -a -o my_robot_run # -a for all topics, -o for output directory
```
This will create a directory (e.g., `my_robot_run/`) containing your bag file (e.g., `my_robot_run_0.db`).

### Playing back a Rosbag

To play back a recorded bag file:
```bash
ros2 bag play my_robot_run/my_robot_run_0.db
```
When a bag is played back, it republishes all recorded messages on their original topics. This means you can launch your analysis nodes (e.g., your `rqt_plot` or your custom debugging scripts) and they will receive the data as if it were coming from a live robot.

### Inspecting a Rosbag

You can inspect the contents of a bag file without playing it back:
```bash
ros2 bag info my_robot_run/my_robot_run_0.db
# Shows topics, message types, message count, duration, etc.

ros2 bag contents my_robot_run/my_robot_run_0.db
# Shows all messages in chronological order (can be very long!)
```

## Leveraging Debugging Tools

Effective use of RQT and Rosbags transforms the debugging of complex robotic systems from a trial-and-error nightmare into a systematic, observable process. They allow you to:
-   **Reproduce Bugs**: Use rosbags to re-run scenarios where a bug occurred.
-   **Analyze Post-Mortem**: Inspect logs and data after a robot failure.
-   **Develop Offline**: Work on perception or control algorithms using recorded data, reducing reliance on expensive hardware.

These tools are your eyes and ears into the distributed mind of your robot, indispensable for any serious robotics development.
