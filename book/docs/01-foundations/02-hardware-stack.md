---
title: "The Hardware Stack: Workstation vs. Edge"
sidebar_label: "Hardware Stack"
---

# The Hardware Stack: Workstation vs. Edge

Building physical AI requires significant computational power, but where that computation happens is a critical architectural decision. The choice between a powerful workstation and a compact edge device defines the capabilities, limitations, and potential applications of your robot.

## The Development Workstation: The Powerhouse

For development, simulation, and training, a powerful desktop workstation is non-negotiable. This is where you will run demanding applications like NVIDIA Isaac Sim, train reinforcement learning models, and analyze large datasets from robot sensors (rosbags).

### Key Components:

-   **GPU (Graphics Processing Unit)**: This is the single most important component. An NVIDIA RTX-series GPU (e.g., RTX 3080, RTX 4090) is essential. The CUDA cores and Tensor Cores within these GPUs are the bedrock of modern AI, accelerating everything from physics simulation to neural network inference.
-   **CPU (Central Processing Unit)**: A multi-core processor (e.g., Intel Core i9, AMD Ryzen 9) is necessary to manage the operating system, development tools, and the Robot Operating System (ROS 2) nodes that don't run on the GPU.
-   **RAM (Random Access Memory)**: 32GB is a comfortable minimum, but 64GB or more is highly recommended, especially when working with large simulations or complex 3D assets.
-   **Storage**: A fast NVMe SSD (Solid State Drive) is crucial for loading large simulation environments and datasets quickly. A minimum of 1TB is advised.
-   **Operating System**: The industry standard for robotics is **Ubuntu Linux (22.04 LTS)**. While other systems can work, nearly all tools and libraries (especially ROS 2) are built and tested primarily on Ubuntu.

:::tip Workstation Role
The workstation is your **foundry**. It's where you forge the AI brain, test it in a perfect simulated world, and iterate on its design a thousand times before ever touching a physical component.
:::

## The Edge Device: The Deployed Brain

Once your AI is trained and your logic is sound, you need to deploy it onto the robot itself. A robot cannot be tethered to a 50-pound workstation. This is the role of the edge device—a small, power-efficient, single-board computer (SBC) that becomes the robot's onboard brain.

### The Gold Standard: NVIDIA Jetson

The NVIDIA Jetson series is the industry leader for embodied AI. These devices pack a power-efficient ARM CPU and a cut-down version of the same NVIDIA GPU architecture found in their desktop cards.

-   **Jetson AGX Orin**: The high-performance option, capable of running complex perception stacks and multiple neural networks simultaneously. This is suitable for autonomous drones, advanced humanoid robots, and sensor-heavy vehicles.
-   **Jetson Orin Nano**: A smaller, more power-efficient and cost-effective option. It's perfect for smaller robots, educational projects, and applications where the AI workload is less intense. It delivers a surprising amount of AI performance in a tiny package.

:::danger The Sim-to-Real Challenge
The primary challenge of this workflow is ensuring the code and models you develop on your powerful workstation can run effectively on the resource-constrained Jetson device. This involves:
-   **Model Optimization**: Using tools like NVIDIA TensorRT to quantize and prune neural networks.
-   **Power Management**: Carefully monitoring and budgeting power consumption.
-   **Cross-Compilation**: Building code on your workstation to be executed on the Jetson's ARM architecture.
:::

Your journey in this handbook will involve both worlds. You will start by building and training on a workstation, and the final chapters will guide you through the process of deploying your creation onto a Jetson device, bringing your digital mind into the physical world.
