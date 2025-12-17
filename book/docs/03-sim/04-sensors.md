---
title: "Simulating Sensors: LiDAR and Depth Cameras"
sidebar_label: "Simulating Sensors"
---

# Simulating Sensors: LiDAR and Depth Cameras

Sensors are a robot's eyes and ears, providing the crucial data needed to perceive its environment and make informed decisions. In simulation, accurately modeling these sensors is paramount to ensuring that algorithms developed in the virtual world transfer effectively to the physical one. This chapter focuses on simulating two common and powerful sensors: **LiDAR (Light Detection and Ranging)** and **Depth Cameras**.

## Simulating LiDAR (Laser Scanners)

LiDAR sensors measure distances to objects by emitting pulsed laser light and calculating the time it takes for the light to return. They are fundamental for tasks like mapping, localization, and obstacle avoidance.

In Gazebo, LiDAR sensors are typically modeled using a plugin that simulates the laser beam's interaction with the environment and publishes the resulting scan data on a ROS 2 topic.

### Key Parameters for LiDAR Simulation:

-   **`topic`**: The ROS 2 topic where scan data will be published (e.g., `/scan`).
-   **`frame_id`**: The coordinate frame of the sensor (e.g., `laser_frame`).
-   **`ray_count`**: The number of laser beams per scan. More rays mean higher resolution but increased computation.
-   **`min_angle` / `max_angle`**: The angular range covered by the laser (e.g., `-M_PI_2` to `M_PI_2` for 180 degrees).
-   **`range_min` / `range_max`**: The minimum and maximum detection range of the sensor.
-   **`update_rate`**: How often the sensor publishes data (e.g., 10 Hz).
-   **`noise`**: Real-world sensors are noisy. Simulating noise (e.g., Gaussian noise) can make your algorithms more robust.

### Gazebo Plugin Example (URDF Xacro Snippet):

```xml
<gazebo reference="laser_link">
  <sensor name="laser" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>      <!-- Number of rays -->
          <resolution>1</resolution>   <!-- Resolution (1 for no interpolation) -->
          <min_angle>-1.5708</min_angle> <!-- -90 degrees -->
          <max_angle>1.5708</max_angle>  <!-- +90 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <argument>--ros-args --remap ~/out:=/scan</argument>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>laser_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

The plugin `libgazebo_ros_ray_sensor.so` will publish `sensor_msgs/msg/LaserScan` messages on the `/scan` topic, allowing your ROS 2 nodes to consume the data just as they would from a real LiDAR.

## Simulating Depth Cameras

Depth cameras (e.g., Intel RealSense, Microsoft Kinect, NVIDIA Isaac Realsense) provide a rich 3D perception by capturing both color (RGB) images and a depth map (distance to pixels). They are invaluable for object detection, 3D reconstruction, and human-robot interaction.

In Gazebo, depth cameras are simulated using a camera plugin that generates both an RGB image and a depth image.

### Key Parameters for Depth Camera Simulation:

-   **`topic_name`**: The base ROS 2 topic for image, camera info, and depth data (e.g., `/camera`).
-   **`camera_name`**: A unique name for the camera.
-   **`horizontal_fov` / `vertical_fov`**: Field of view.
-   **`image_width` / `image_height`**: Resolution of the image.
-   **`near` / `far`**: Clipping planes for depth sensing.
-   **`update_rate`**: Data publishing frequency.
-   **`distortion_k1`, `distortion_k2`, `distortion_t1`, `distortion_t2`**: Parameters to simulate lens distortion, making the simulation more realistic.

### Gazebo Plugin Example (URDF Xacro Snippet):

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="depth">
    <pose>0 0 0 0 0 0</pose>
    <always_on>1</always_on>
    <visualize>true</visualize>
    <update_rate>30.0</update_rate>
    <camera name="rgbd_camera">
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_depth_camera.so">
      <ros>
        <argument>--ros-args --remap ~/depth/image_raw:=/camera/depth/image_raw --remap ~/depth/points:=/camera/depth/points --remap ~/image_raw:=/camera/rgb/image_raw --remap ~/camera_info:=/camera/rgb/camera_info</argument>
      </ros>
      <camera_name>rgbd_camera</camera_name>
      <frame_name>camera_depth_frame</frame_name>
      <min_depth>0.1</min_depth>
      <max_depth>10.0</max_depth>
      <point_cloud_cutout_max_depth>0.1</point_cloud_cutout_max_depth>
      <point_cloud_cutout_min_depth>10.0</point_cloud_cutout_min_depth>
      <point_cloud_cutout_angle_min>0.1</point_cloud_cutout_angle_min>
      <point_cloud_cutout_angle_max>10.0</point_cloud_cutout_angle_max>
      <point_cloud_cutoff_fuzz>0.1</point_cloud_cutoff_fuzz>
    </plugin>
  </sensor>
</gazebo>
```

The `libgazebo_ros_depth_camera.so` plugin will publish data on topics like `/camera/rgb/image_raw` (for the color image), `/camera/depth/image_raw` (for the depth map), and `/camera/depth/points` (for the point cloud), formatted as `sensor_msgs/msg/Image` and `sensor_msgs/msg/PointCloud2`.

By carefully configuring these sensor plugins, you can create a simulated perception system that closely mirrors your physical robot, allowing for robust development and testing of vision and navigation algorithms.
