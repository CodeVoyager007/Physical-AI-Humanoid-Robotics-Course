---
title: "Modeling Bipeds: Designing a Humanoid Linkage"
sidebar_label: "Modeling Bipeds"
---

# Modeling Bipeds: Designing a Humanoid Linkage

Designing a humanoid robot is a complex endeavor, beginning with its fundamental mechanical structure—the linkage. Unlike wheeled robots, bipeds introduce unique challenges in balance, stability, and locomotion. This chapter delves into the principles of modeling humanoid robot linkages using URDF, focusing on the key considerations for bipedal design.

## Understanding Humanoid Kinematics

Humanoids are characterized by their anthropomorphic structure, typically featuring two legs, a torso, two arms, and a head. This design mimics the human body, allowing for interaction with human-centric environments. The kinematics of a bipedal robot refer to the study of its motion without considering the forces that cause it.

### Degrees of Freedom (DoF)

The complexity of a humanoid is often measured by its Degrees of Freedom (DoF). Each independent joint movement adds a DoF. A typical human body has over 200 DoF. For robotics, we simplify this significantly, often aiming for 20-40 DoF for capable humanoids.

-   **Legs**: Crucial for locomotion and balance. Each leg typically has 5-6 DoF (hip roll, pitch, yaw; knee pitch; ankle pitch, roll).
-   **Torso/Waist**: Provides upper body mobility and helps shift the center of mass. Usually 1-3 DoF.
-   **Arms**: For manipulation and interaction. Each arm can have 6-7 DoF (shoulder roll, pitch, yaw; elbow pitch; wrist pitch, roll, yaw).
-   **Head/Neck**: For perception and expression. 2-3 DoF.

## Key Considerations in Bipedal URDF Design

When translating a humanoid concept into a URDF, several factors are paramount:

### 1. Mass Distribution and Center of Mass (CoM)

-   **Balance**: For stable standing and walking, the robot's Center of Mass (CoM) must project onto its support polygon (the area on the ground enclosed by its feet).
-   **Inertial Properties**: Each link's mass and inertia tensor are crucial for physics simulations. Accurate values are necessary for realistic dynamics. Heavy components should be placed low and centrally to improve stability.

### 2. Joint Limits and Ranges of Motion

-   **Human-like Motion**: Joints must be configured with appropriate limits to prevent self-collision and unnatural movements. These limits directly impact the robot's reachable workspace and gait.
-   **Safety**: Realistic joint limits also prevent the robot from damaging itself or its environment in physical deployments.

### 3. Actuation and Strength

-   While URDF primarily describes geometry and inertia, the `limit` tags in joints (`velocity` and `effort`) provide initial estimations for required actuator strength. These values will later inform the choice of motors.

### 4. Self-Collision Awareness

-   **Collision Geometries**: Simplify collision geometries for performance, but ensure they accurately represent the physical boundaries of the links to prevent simulated self-collisions.
-   **Robot Description**: The URDF defines the nominal configuration, but control software must be aware of self-collision possibilities.

## Example: Simplified Humanoid Leg Segment (Xacro)

Let's look at how Xacro simplifies the definition of a repeating structure like a leg segment.

```xml
<?xml version="1.0"?>
<robot name="simple_biped" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:property name="thigh_mass" value="2.0" />
  <xacro:property name="thigh_length" value="0.3" />
  <xacro:property name="thigh_radius" value="0.04" />

  <xacro:property name="shank_mass" value="1.5" />
  <xacro:property name="shank_length" value="0.3" />
  <xacro:property name="shank_radius" value="0.03" />

  <xacro:macro name="leg_segment" params="prefix parent_link child_link origin_xyz origin_rpy joint_type axis_xyz length mass radius">
    <link name="${child_link}">
      <visual>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="${mass/12*(3*radius*radius + length*length)}" ixy="0" ixz="0"
                 iyy="${mass/12*(3*radius*radius + length*length)}" iyz="0"
                 izz="${mass/2*radius*radius}"/>
      </inertial>
    </link>

    <joint name="${parent_link}_to_${child_link}_joint" type="${joint_type}">
      <parent link="${parent_link}"/>
      <child link="${child_link}"/>
      <origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
      <axis xyz="${axis_xyz}"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/> <!-- Example limits -->
    </joint>
  </xacro:macro>

  <!-- Example usage: Connecting torso to hip -->
  <link name="torso"/> <!-- Assume torso exists -->

  <xacro:leg_segment
    prefix="right"
    parent_link="torso"
    child_link="right_thigh_link"
    origin_xyz="0 -0.1 0"
    origin_rpy="0 0 0"
    joint_type="revolute"
    axis_xyz="1 0 0"
    length="${thigh_length}"
    mass="${thigh_mass}"
    radius="${thigh_radius}"
  />

</robot>
```

Modeling bipeds in URDF is an iterative process. Start with a simplified model, focus on getting the kinematics and inertial properties correct, and then gradually add detail. Accurate URDFs are the foundation for stable gait generation, reliable control, and effective simulation of humanoid robots.
