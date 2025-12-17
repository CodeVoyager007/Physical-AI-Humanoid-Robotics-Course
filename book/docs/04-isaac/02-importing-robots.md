---
title: "Importing Robots into Omniverse: From URDF to USD"
sidebar_label: "Importing Robots"
---

# Importing Robots into Omniverse: From URDF to USD

The Universal Robot Description Format (URDF) is the standard for describing robots in ROS 2. However, NVIDIA Omniverse and Isaac Sim primarily use **USD (Universal Scene Description)**. To bring your ROS-defined robot into Isaac Sim, you need to import your URDF and convert it into a USD asset. This chapter guides you through that essential process.

## Understanding USD in Isaac Sim

USD is a powerful, extensible framework for composing, modeling, simulating, and rendering 3D scenes. It's not just a file format but a complete ecosystem for 3D content creation. In Isaac Sim, your robot, its environment, sensors, and any other assets are all represented as USD primitives.

Key concepts of USD in Isaac Sim:
-   **Composition**: USD layers allow non-destructive editing and collaboration. You can layer changes on top of a base asset without modifying the original.
-   **Prims**: The fundamental building blocks in USD, analogous to nodes in a scene graph. Each prim can have properties (attributes) and relationships to other prims.
-   **Schema**: USD defines various schemas for different types of objects (e.g., `Sphere`, `Capsule`, `Mesh` for geometry; `PhysicsMeshCollision` for collision properties). Isaac Sim extends USD with schemas for robots, joints, sensors, and actuators.

## The URDF Importer Tool

Isaac Sim provides a dedicated **URDF Importer** extension to facilitate the conversion of your URDF files into USD assets. This tool handles the kinematic structure, visual and collision geometries, and inertial properties defined in your URDF.

### Steps to Import a URDF:

1.  **Launch Isaac Sim**: Start NVIDIA Isaac Sim.
2.  **Enable URDF Importer Extension**: Go to `Window > Extensions` and ensure the "URDF Importer" extension is enabled.
3.  **Open URDF Importer**: Go to `Robotics > URDF Importer`.
4.  **Select URDF File**: Browse to your robot's `.urdf` file (or `.urdf.xacro` if you are using Xacro, as the importer will process it first).
5.  **Configuration Options**:
    -   **Fix Base Link**: If your robot's base is floating, you might want to fix it to the world (e.g., for a static manipulator).
    -   **Merge Fixed Joints**: Optimizes the USD by merging links connected by fixed joints, reducing the number of prims.
    -   **Self Collision**: Enable this if you want to generate self-collision pairs for collision detection within the robot.
    -   **Convex Decomposition**: For complex mesh collision geometries, this option automatically generates simplified convex hulls for better physics performance.
    -   **Prim Path**: The path in the USD stage where your robot will be imported (e.g., `/World/my_robot`).
6.  **Import**: Click the "Import" button.

Isaac Sim will then generate a USD representation of your robot in the stage. You should see your robot model appear in the viewport and its structure in the "Stage" window (USD tree view).

:::tip Post-Import Adjustments
After importing, you might need to:
-   **Adjust Materials**: The importer attempts to apply basic materials, but you may want to refine them for visual fidelity using Omniverse's Material Editor.
-   **Add ROS 2 Components**: Manually add ROS 2 publishers and subscribers to your robot's joints or sensors if they aren't automatically configured by the importer (depending on the complexity of your URDF's Gazebo plugins).
-   **Add Controllers**: Attach appropriate controllers (e.g., `DifferentialController` for a wheeled robot, `ArticulationController` for a manipulator) to enable control of your robot's joints via ROS 2.
:::

## Direct USD Export (Advanced)

For more complex robot definitions or to leverage advanced USD features not fully covered by the URDF Importer, you can:
-   **Manually build USD**: Create your robot directly using USD prims and properties within Omniverse applications.
-   **Export from CAD**: Use Omniverse Connectors to export directly from CAD software (like SolidWorks, Fusion 360) to USD, preserving assembly structure, materials, and physics properties.

The transition from URDF to USD is a key step in leveraging Isaac Sim's advanced capabilities. By successfully importing your robot, you unlock a powerful new environment for simulation, synthetic data generation, and reinforcement learning.
