---
title: "URDF Basics: Unified Robot Description Format"
sidebar_label: "URDF Basics"
---

# URDF Basics: Unified Robot Description Format

Before we can simulate a robot or even interact with a physical one, we need a way to describe it digitally. The **Unified Robot Description Format (URDF)** is an XML-based file format used in ROS 2 to describe all the physical and kinematic properties of a robot. It's the robot's DNA, telling simulation environments and control software everything they need to know.

## What does URDF describe?

A URDF file typically contains descriptions of:
-   **Links**: The rigid bodies of the robot (e.g., base, arm segments, wheels). Each link has physical properties like mass, inertia, and visual/collision geometries.
-   **Joints**: The connections between links, allowing them to move relative to each other. Joints have properties like type (revolute, prismatic, fixed), axis of rotation, limits, and dynamics.

## Structure of a URDF File

A URDF file starts with a `<robot>` tag, and inside it, you define all your links and joints.

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- =============================== -->
  <!--        LINK DEFINITIONS         -->
  <!-- =============================== -->

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.02" radius="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.02" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- More links for right_wheel, etc. -->

  <!-- =============================== -->
  <!--       JOINT DEFINITIONS         -->
  <!-- =============================== -->

  <joint name="base_to_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.0 -0.06 0.0" rpy="1.57079632679 0 0"/> <!-- Rotate so wheel is upright -->
    <axis xyz="0 0 1"/> <!-- Axis of rotation -->
  </joint>

  <!-- More joints for base_to_right_wheel, etc. -->

  <!-- =============================== -->
  <!--       GAZEBO PROPERTIES         -->
  <!-- =============================== -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- You might add transmission and controller definitions here too for real robots -->

</robot>
```

## Links (`<link>`)

Each `<link>` tag defines a rigid body.

-   **`name`**: A unique identifier for the link.
-   **`visual`**: Defines how the link looks.
    -   `geometry`: Shape (box, cylinder, sphere) or mesh (`.dae`, `.stl`).
    -   `material`: Color or texture.
-   **`collision`**: Defines the shape used for collision detection. Often simpler than the visual geometry for performance.
-   **`inertial`**: Physical properties crucial for physics simulation.
    -   `mass`: Mass of the link.
    -   `inertia`: 3x3 rotational inertia matrix. This is critical for realistic dynamics.

## Joints (`<joint>`)

Each `<joint>` tag defines how two links are connected.

-   **`name`**: Unique identifier for the joint.
-   **`type`**:
    -   `revolute`: Rotational joint with a limited range.
    -   `continuous`: Rotational joint with unlimited range (e.g., wheel).
    -   `prismatic`: Linear joint with a limited range.
    -   `fixed`: No movement, effectively combines two links into one rigid body.
-   **`parent` / `child`**: Specifies the two links connected by the joint.
-   **`origin`**: Defines the joint's position and orientation relative to the parent link.
    -   `xyz`: Position (x, y, z).
    -   `rpy`: Roll, Pitch, Yaw rotation (in radians).
-   **`axis`**: Defines the axis of rotation or translation for revolute, continuous, and prismatic joints.
-   **`limit`**: For `revolute` and `prismatic` joints, defines the upper and lower limits, velocity limits, and effort limits.

## Xacro: Simplifying URDF

Writing complex robots in pure URDF can be very repetitive and hard to manage. **Xacro (XML Macros)** is an XML macro language that allows you to use variables, mathematical expressions, and macros to generate URDF files. This dramatically reduces duplication and improves readability.

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Define properties as xacro properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.03" />
  <xacro:property name="wheel_width" value="0.02" />

  <!-- Define a macro for a wheel -->
  <xacro:macro name="wheel" params="prefix parent_link x_coord y_coord">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder length="${wheel_width}" radius="${wheel_radius}"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
      </inertial>
    </link>

    <joint name="${parent_link}_to_${prefix}_wheel" type="continuous">
      <parent link="${parent_link}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x_coord} ${y_coord} 0.0" rpy="${M_PI/2} 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Base Link definition (same as before) -->
  <link name="base_link">...</link>

  <!-- Use the wheel macro -->
  <xacro:wheel prefix="left" parent_link="base_link" x_coord="0.0" y_coord="-0.06" />
  <xacro:wheel prefix="right" parent_link="base_link" x_coord="0.0" y_coord="0.06" />

</robot>
```
To process an Xacro file into a standard URDF:
```bash
ros2 run xacro xacro my_robot.urdf.xacro > my_robot.urdf
```

URDF is your robot's blueprint. A well-defined URDF is critical for accurate simulation, effective control, and proper visualization in tools like RViz. Mastering it is the first step in bringing your robot designs to life.
