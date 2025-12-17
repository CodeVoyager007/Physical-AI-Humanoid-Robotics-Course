---
title: "Unity Bridge: High-Fidelity Visualization with Unity"
sidebar_label: "Unity Bridge"
---

# Unity Bridge: High-Fidelity Visualization with Unity

While Gazebo excels at physics-accurate simulation, its visualization capabilities are often secondary to its physics engine. For creating stunning, high-fidelity robot renderings, complex environments, or interactive user interfaces, game engines like Unity offer unparalleled power. The **Unity Robotics Hub** provides tools and packages that bridge the gap between ROS 2 and Unity, allowing you to leverage Unity for advanced visualization, human-robot interaction (HRI) development, and even training AI models.

## Why use Unity with ROS 2?

-   **High-Fidelity Graphics**: Unity's rendering capabilities allow for photorealistic environments and detailed robot models, which is crucial for perception algorithm development (e.g., training vision models with simulated data).
-   **Interactive Environments**: Create dynamic, interactive scenes with advanced lighting, particle effects, and diverse assets, surpassing what's easily achievable in Gazebo.
-   **User Interfaces**: Develop intuitive and engaging human-robot interfaces directly within Unity.
-   **Cross-Platform Deployment**: Export Unity applications to various platforms (desktop, web, mobile, VR/AR), enabling wider deployment of HRI or robot teleoperation interfaces.
-   **Synthetic Data Generation**: Unity can be used to generate vast amounts of labeled synthetic data for training machine learning models, especially for computer vision tasks where real-world data collection is expensive or impractical.

## The Unity Robotics Hub

The Unity Robotics Hub is a collection of resources, tutorials, and packages designed to facilitate the integration of Unity with robotics ecosystems. Key components include:

### 1. ROS-TCP-Connector

This package enables communication between a Unity application and a ROS 2 graph over TCP. It allows Unity to act as a ROS 2 node, publishing and subscribing to topics, and calling services.

-   **How it works**: The `ROS-TCP-Connector` acts as a proxy, translating ROS 2 messages to a Unity-compatible format and vice-versa. It uses a custom server within the ROS 2 environment (`ros_tcp_endpoint` node) that handles the TCP connection to the Unity application.
-   **Installation**:
    -   In your ROS 2 workspace, install `ros_tcp_endpoint`:
        ```bash
        sudo apt update
        rosdep install -i --from-path src --rosdistro humble -y
        colcon build --packages-select ros_tcp_endpoint
        ```
    -   In Unity, install the `ROS-TCP-Connector` package via the Unity Package Manager (UPM) from its Git URL.

### 2. ROS-Unity-Integration

This meta-package provides additional tools and examples for integrating ROS 2 with Unity, including:

-   **URDF Importer**: A tool to import URDF files directly into Unity, automatically generating a robot model with its joints and links. This is a game-changer for bringing your existing robot definitions into Unity.
-   **Message Generation**: Automatically generates C# message types from your ROS 2 `.msg`, `.srv`, and `.action` definitions, so you can use them directly in your Unity scripts.
-   **Example Scenes**: Demonstrations of how to set up publishers, subscribers, and services in Unity.

## Workflow: Using Unity for Visualization

1.  **URDF Import**: Import your robot's URDF (or Xacro-generated URDF) into Unity using the URDF Importer. This creates a Unity GameObject representation of your robot.
2.  **ROS 2 Node in Unity**: Configure a Unity GameObject to run a `ROS-TCP-Connector` component. This component acts as your Unity-based ROS 2 node.
3.  **Subscribe to Robot State**: Subscribe to ROS 2 topics like `/joint_states` (published by your Gazebo simulation or physical robot) to update the joint positions of your Unity robot model in real-time.
4.  **Publish Commands**: Publish ROS 2 topics, such as `/cmd_vel` or joint commands, from Unity to control your robot in Gazebo or the physical world.
5.  **Visualize Sensor Data**: Subscribe to sensor topics (e.g., `/camera/rgb/image_raw`, `/scan`) and display them within Unity for advanced visualization.

### Example: Visualizing Joint States in Unity

Imagine you have a Gazebo simulation publishing `/joint_states`. Your Unity application would:
1.  Have a `ROSConnection` component to manage the TCP connection.
2.  Create a C# script with a `ROSConnection.Subscribe` call to the `/joint_states` topic.
3.  In the subscription callback, parse the `JointState` message.
4.  Update the corresponding joint GameObjects in Unity with the new positions.

```csharp
// Example C# script in Unity attached to your robot GameObject
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // Assuming JointStateMsg is generated

public class JointStateSubscriber : MonoBehaviour
{
    ROSConnection ros;
    public GameObject robotRoot; // Assign your robot's root GameObject in Inspector

    // Dictionary to hold references to all joint transforms
    private Dictionary<string, Transform> jointTransforms = new Dictionary<string, Transform>();

    void Start()
    {
        ros = ROSConnection.Get  Instance();
        ros.Subscribe<JointStateMsg>("/joint_states", ReceiveJointStates);

        // Populate jointTransforms dictionary (e.g., recursively find all joints)
        PopulateJointTransforms(robotRoot.transform);
    }

    void PopulateJointTransforms(Transform parent)
    {
        foreach (Transform child in parent)
        {
            // Assuming your joint GameObjects are named after their ROS joint names
            jointTransforms[child.name] = child;
            PopulateJointTransforms(child);
        }
    }

    void ReceiveJointStates(JointStateMsg jointState)
    {
        for (int i = 0; i < jointState.name.Length; i++)
        {
            string jointName = jointState.name[i];
            float position = (float)jointState.position[i]; // Assuming single axis rotation

            if (jointTransforms.ContainsKey(jointName))
            {
                // Example: Update rotation for a hinge joint
                // You'll need more complex logic for different joint types (e.g., prismatic)
                // and to convert ROS radians to Unity degrees, handle local vs global
                float rotationDegree = position * Mathf.Rad2Deg;
                jointTransforms[jointName].localRotation = Quaternion.Euler(rotationDegree, 0, 0); // Adjust axis
            }
        }
    }
}
```

By bridging ROS 2 and Unity, you unlock a new dimension of possibilities for robotic development, combining the robustness of ROS with the visual and interactive power of a modern game engine.
