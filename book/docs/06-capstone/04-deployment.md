---
title: "Deployment: Flashing to Jetson Orin Nano"
sidebar_label: "Deployment"
---

# Deployment: Flashing to Jetson Orin Nano

After developing and extensively testing your AI and robotics algorithms in simulation, the ultimate goal is often to deploy them onto a physical robot. For many embodied AI applications, especially those requiring significant on-device AI inference, **NVIDIA Jetson Orin Nano** is an ideal edge platform. This chapter guides you through the process of preparing your Jetson Orin Nano and deploying your ROS 2-based AI applications.

## The Jetson Orin Nano: Your Robot's Brain

The Jetson Orin Nano Developer Kit is a small, powerful, and energy-efficient platform designed for AI at the edge. It combines a powerful ARM-based CPU with a 1024-core NVIDIA Ampere architecture GPU, specifically engineered for AI workloads.

### Key Features:
-   **Ampere GPU**: Provides the computational muscle for AI inference (e.g., object detection, VSLAM).
-   **ARM CPU**: Handles general-purpose computing, running the Linux OS, and ROS 2 nodes.
-   **Low Power Consumption**: Designed for battery-powered robots.
-   **NVIDIA JetPack SDK**: A comprehensive software stack including Linux for Tegra (L4T), CUDA, cuDNN, TensorRT, and ROS packages.

## Flashing JetPack onto Jetson Orin Nano

The first step is to flash the NVIDIA JetPack SDK onto your Jetson Orin Nano's storage. JetPack includes the OS (Ubuntu Linux), NVIDIA drivers, AI libraries, and developer tools.

### Prerequisites:
-   A host PC with Ubuntu 18.04/20.04/22.04 LTS.
-   NVIDIA SDK Manager installed on the host PC.
-   Micro-USB cable for flashing, or network connection for over-the-air update.
-   (For Developer Kit) 64GB+ microSD card, or NVMe SSD for production modules.

### Steps (using NVIDIA SDK Manager):

1.  **Install SDK Manager**: Download and install NVIDIA SDK Manager on your Ubuntu host PC.
    ```bash
    sudo apt install ./sdkmanager_[version].deb
    ```
2.  **Launch SDK Manager**:
    ```bash
    sdkmanager
    ```
    Log in with your NVIDIA Developer account.
3.  **Select Hardware**: In SDK Manager, select "Jetson Orin Nano Developer Kit" (or your specific Jetson module).
4.  **Select JetPack Version**: Choose the latest stable JetPack version.
5.  **Download and Prepare**: SDK Manager will download the necessary components.
6.  **Flash Target OS**:
    -   Follow the instructions in SDK Manager to put your Jetson Orin Nano into recovery mode.
    -   Connect it to your host PC via Micro-USB.
    -   Click "Flash" in SDK Manager. This will install the OS and all necessary drivers/libraries.
7.  **Post-Flash Setup**: After flashing, connect a monitor, keyboard, and mouse to the Jetson, boot it up, and complete the initial Ubuntu setup (create user, set timezone, etc.).

## Deploying ROS 2 Applications

Once JetPack is installed and configured, you have a powerful platform ready for your ROS 2 applications.

### 1. Setting up ROS 2 on Jetson

JetPack typically includes pre-built ROS 2 binaries, but you might need to install additional packages or build from source if you have custom messages or specific versions.

```bash
# Install ROS 2 Humble (example)
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop # Or ros-humble-ros-base for minimal installation
```

### 2. Transferring Your ROS 2 Workspace

You can transfer your ROS 2 workspace (which contains your nodes, packages, and launch files) to the Jetson using `rsync`, `scp`, or by simply cloning your Git repository.

```bash
# Example using rsync from host PC to Jetson
rsync -avz --exclude 'build' --exclude 'install' --exclude 'log' ~/ros2_ws/ ubuntu@<jetson_ip_address>:~/ros2_ws/
```

### 3. Building and Running on Jetson

After transferring, build your workspace on the Jetson:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Then, you can source your workspace and run your applications:

```bash
source install/setup.bash
ros2 launch my_robot_package my_robot_bringup.launch.py
```

## Optimizing for Jetson

-   **TensorRT**: Leverage NVIDIA TensorRT to optimize your deep learning models for inference on the Jetson GPU. This can significantly improve performance and reduce latency.
-   **Quantization**: Convert your models to lower precision (e.g., FP16 or INT8) to further reduce memory footprint and increase inference speed.
-   **Power Modes**: The Jetson supports various power modes. Select the appropriate mode for your application to balance performance and energy consumption.
-   **Monitor Performance**: Use tools like `tegrastats` or `jtop` (install `pip3 install jetson-stats`) to monitor CPU, GPU, and memory usage on your Jetson.

Deploying to the Jetson Orin Nano brings your embodied AI creations to life. It's the critical step where your algorithms move from the safety of simulation to the challenges and rewards of interacting with the real world.
