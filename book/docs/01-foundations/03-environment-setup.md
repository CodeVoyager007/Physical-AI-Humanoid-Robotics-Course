---
title: "Environment Setup: Ubuntu and Docker"
sidebar_label: "Environment Setup"
---

# Environment Setup: Ubuntu and Docker

A standardized and reproducible development environment is the bedrock of any successful robotics project. In this chapter, we will configure the two most critical components of our software stack: Ubuntu 22.04 LTS and Docker.

## Why Ubuntu 22.04 LTS?

The Robot Operating System (ROS) and the broader robotics community have standardized on Ubuntu Linux. While it's possible to use other operating systems, you will encounter significantly more friction.

-   **LTS (Long-Term Support)**: The "LTS" release of Ubuntu provides five years of security and maintenance updates. For robotics, stability is paramount. The 22.04 release (codenamed "Jammy Jellyfish") is the primary target for modern ROS 2 (Iron Irwini) and NVIDIA Isaac Sim.
-   **Compatibility**: Nearly all robotics software, drivers, and libraries are tested and released for specific Ubuntu LTS versions. Using 22.04 ensures you spend your time building robots, not debugging compatibility issues.

### Installation

1.  **Download**: Get the Ubuntu 22.04 LTS Desktop image from the [official Ubuntu website](https://ubuntu.com/download/desktop).
2.  **Create a Bootable USB**: Use a tool like [BalenaEtcher](https://www.balena.io/etcher/) to flash the downloaded ISO image to a USB drive (8GB or larger).
3.  **Install**: Boot your workstation from the USB drive and follow the installation prompts. We recommend the "Minimal installation" to keep your system clean, and be sure to check the option to "Install third-party software for graphics and Wi-Fi hardware."

## Why Docker?

Robotics software is notoriously complex, with a web of specific library versions and system dependencies. Managing these on your host OS can quickly lead to a state known as "dependency hell." Docker solves this by providing containerization.

:::tip Key Concept: Containers
A container is a lightweight, standalone, executable package of software that includes everything needed to run it: code, runtime, system tools, system libraries, and settings. It isolates the application from its environment, ensuring that it works uniformly despite differences between development and staging.
:::

We will use Docker for several key purposes:
1.  **Isolation**: To run ROS 2 and NVIDIA Isaac Sim in a clean, controlled environment without "polluting" our host Ubuntu system.
2.  **Reproducibility**: To ensure that the environment we develop in is identical to the one our colleagues use and, eventually, the one we deploy on the robot.
3.  **Dependency Management**: To encapsulate complex dependencies required by tools like Isaac Sim, which may conflict with other software on the system.

### Installation

We will install Docker Engine from Docker's official `apt` repository.

1.  **Set up the repository**:
    ```bash
    sudo apt-get update
    sudo apt-get install -y ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update
    ```

2.  **Install Docker Engine**:
    ```bash
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```

3.  **Post-installation steps (Manage Docker as a non-root user)**:
    This is a critical step for convenience and security.
    ```bash
    sudo groupadd docker
    sudo usermod -aG docker $USER
    ```
    :::danger Important
    You must **log out and log back in** for this group membership change to take effect.
    :::

4.  **Verify the installation**:
    After logging back in, test that Docker is working without `sudo`.
    ```bash
    docker run hello-world
    ```
    This command should download and run a small test image, printing a confirmation message.

With Ubuntu 22.04 and Docker installed, your workstation is now a prepared canvas, ready for the complex and powerful robotics software we will install in the coming chapters.
