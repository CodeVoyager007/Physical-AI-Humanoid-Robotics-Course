---
title: "Gazebo Physics: Gravity, Collision, and Inertia"
sidebar_label: "Gazebo Physics"
---

# Gazebo Physics: Gravity, Collision, and Inertia

Gazebo is a powerful 3D robotics simulator that allows you to accurately test robot designs and algorithms in a virtual environment. It provides a robust physics engine that models real-world phenomena like gravity, friction, collision, and inertia. Understanding how Gazebo interprets and applies these physics concepts is crucial for creating realistic and reliable simulations.

## The Gazebo World

A Gazebo simulation takes place in a "world" file, typically an `.sdf` (Simulation Description Format) or `.world` file. This file defines the environment, including static objects (walls, ground plane), dynamic objects, sensors, and, crucially, the physics engine parameters.

### Default Physics Engine

Gazebo supports various physics engines, with `ODE` (Open Dynamics Engine) being the default. Others include `Bullet`, `DART`, and `Simbody`. While they offer different levels of accuracy and performance, the core principles of how Gazebo applies physics remain consistent.

## Gravity

Gravity is a fundamental force in any physical simulation. Gazebo applies a constant gravitational acceleration to all links within the simulated world.

-   **Configuration**: Gravity can be configured in the world file. The default for Earth is `9.8 m/s^2` in the negative Z direction.
    ```xml
    <world name="default">
      <gravity>0 0 -9.8</gravity>
      <!-- Other world elements -->
    </world>
    ```
-   **Impact**: Correctly modeling mass and inertia (see below) is essential for how links respond to gravity, affecting balance, joint loads, and overall stability.

## Collision Detection

Collision detection is how Gazebo determines when two physical objects are in contact. This is critical for preventing robots from passing through walls or self-colliding.

-   **Collision Geometries**: In URDF, each `<link>` can have a `<collision>` tag. This tag defines a simplified geometric shape (box, cylinder, sphere) that the physics engine uses for collision calculations.
    ```xml
    <link name="base_link">
      <collision>
        <geometry>
          <box size="0.2 0.1 0.05"/>
        </geometry>
      </collision>
    </link>
    ```
-   **Performance**: Collision geometries should be as simple as possible while still accurately representing the physical extent of the object. Using complex meshes for collision will significantly slow down the simulation.
-   **Friction**: When objects collide, friction models how they resist relative motion. This is typically defined as properties of the materials in the SDF world file.
    -   `mu1`: Coefficient of static friction in one direction.
    -   `mu2`: Coefficient of static friction in the perpendicular direction.
    -   `kp`: Spring stiffness coefficient for contact.
    -   `kd`: Damping coefficient for contact.

## Inertia: Mass, Center of Mass, and Inertia Tensor

Inertial properties describe how a body resists changes to its motion. Accurate inertial properties are perhaps the most critical factor for realistic dynamics in Gazebo.

-   **Mass**: Defined in the `<inertial>` tag of a `<link>`.
    ```xml
    <link name="my_link">
      <inertial>
        <mass value="1.0"/> <!-- Mass in kilograms -->
        <!-- ... inertia tensor ... -->
      </inertial>
    </link>
    ```
-   **Center of Mass (CoM)**: Also defined within the `<inertial>` tag via the `<origin>` element. This specifies the link's center of mass relative to its joint origin.
    ```xml
    <link name="my_link">
      <inertial>
        <origin xyz="0 0 0.02" rpy="0 0 0"/> <!-- CoM 2cm above joint origin -->
        <mass value="1.0"/>
        <!-- ... inertia tensor ... -->
      </inertial>
    </link>
    ```
    -   **Impact**: An incorrectly placed CoM can lead to unstable robots that fall over easily or exhibit unrealistic movements.

-   **Inertia Tensor**: A 3x3 matrix (`ixx ixy ixz iyy iyz izz`) that describes how a body's mass is distributed around its CoM. It dictates how easy or hard it is to rotate a body around its principal axes.
    ```xml
    <link name="my_link">
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>
    ```
    -   **Calculation**: For simple geometric shapes (boxes, cylinders, spheres), inertia tensors can be calculated using standard formulas. For complex meshes, CAD software often provides these values. Incorrect inertia values are a common source of unrealistic simulation behavior.

## Simulating Actuators and Sensors

While URDF defines the robot, Gazebo is where you add more complex elements like:
-   **Actuators (Motors)**: These are typically controlled by Gazebo plugins that interface with ROS 2 controllers. You define their properties (e.g., torque limits, velocity limits) and connect them to the robot's joints.
-   **Sensors**: Gazebo provides a wide range of simulated sensors (cameras, LiDAR, IMU, sonar). You add them to your URDF or SDF and configure their properties, and they will publish data on ROS 2 topics, just like real sensors.

Mastering Gazebo's physics settings, particularly the accurate representation of mass, collision geometries, and inertia, is fundamental to creating simulations that are not just visually appealing but also physically meaningful. This allows your robot's algorithms to be developed and tested in a virtual environment that closely mirrors the real world.
