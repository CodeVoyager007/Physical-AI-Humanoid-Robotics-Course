---
title: "Linux for Robotics: Essential CLI Skills"
sidebar_label: "Linux for Robotics"
---

# Linux for Robotics: Essential CLI Skills

In robotics, the Command Line Interface (CLI) is not just a tool for developers; it is the primary environment for interacting with, debugging, and managing the robot. While modern graphical user interfaces (GUIs) exist, fluency in the CLI is essential for efficiency and for accessing the full power of the Linux and ROS ecosystems.

This chapter is not an exhaustive Linux tutorial but a focused guide on the commands and concepts you will use daily as a robotics engineer.

## The Shell: Your Primary Interface

The shell is the program that takes your commands and tells the operating system what to do. The default shell in Ubuntu is **Bash** (`Bourne Again SHell`).

## 1. Navigating the Filesystem

You are always "in" a directory. Knowing where you are and how to move is fundamental.

-   **`pwd` (Print Working Directory)**: Shows your current location.
    ```bash
    pwd
    # Output: /home/ubuntu/ros2_ws
    ```
-   **`ls` (List)**: Lists the contents of the current directory.
    -   `ls -l`: Lists in long format, showing permissions, owner, size, and modification date.
    -   `ls -a`: Shows all files, including hidden ones (those starting with a `.`).
    -   `ls -lh`: A common combination: long format and human-readable file sizes (e.g., `4.0K`, `1.2M`).
-   **`cd` (Change Directory)**: Moves you to another directory.
    -   `cd /path/to/directory`: Absolute path from the root (`/`).
    -   `cd my_folder`: Relative path to a folder inside your current directory.
    -   `cd ..`: Move up one level.
    -   `cd ~` or `cd`: Go to your home directory.

## 2. Managing Files and Directories

-   **`touch <filename>`**: Creates an empty file.
-   **`mkdir <dirname>`**: Creates a new directory.
    -   `mkdir -p path/to/nested/dir`: Creates parent directories as needed.
-   **`cp <source> <destination>`**: Copies a file or directory.
    -   `cp file.txt file_backup.txt`
    -   `cp -r my_folder/ my_folder_backup/`: The `-r` (recursive) flag is required for directories.
-   **`mv <source> <destination>`**: Moves or renames a file or directory.
    -   `mv old_name.txt new_name.txt` (rename)
    -   `mv file.txt my_folder/` (move)
-   **`rm <filename>`**: Removes a file.
    -   `rm -r my_folder/`: Recursively removes a directory and all its contents.
    :::danger Use `rm -r` with extreme caution
    There is no "trash can" in the default CLI. Once a file is deleted with `rm`, it is gone forever. Double-check your commands before running them.
    :::

## 3. Viewing and Editing Files

-   **`cat <filename>`**: Dumps the entire content of a file to the screen. Good for short files.
-   **`less <filename>`**: A pager that lets you view large files. Use arrow keys to navigate, and press `q` to quit.
-   **`head <filename>`**: Shows the first 10 lines. `head -n 20` shows the first 20.
-   **`tail <filename>`**: Shows the last 10 lines. `tail -n 20` shows the last 20.
    -   `tail -f <logfile>`: The `-f` (follow) flag is invaluable for watching log files in real-time.

## 4. System and Process Management

-   **`top` / `htop`**: An interactive task manager. Shows CPU and memory usage. `htop` is a more user-friendly version (`sudo apt-get install htop`).
-   **`ps aux`**: Gives a static snapshot of all running processes.
-   **`kill <PID>`**: Terminates a process. You get the Process ID (PID) from `top` or `ps`.
-   **`df -h`**: Shows disk free space in a human-readable format.
-   **`free -h`**: Shows free memory in a human-readable format.

## 5. The Superpower: Chaining Commands with Pipes

The pipe `|` is one of the most powerful concepts in Linux. It allows you to send the output of one command to the input of another.

-   **`grep` (Global Regular Expression Print)**: Searches for patterns in text.
    Imagine you want to find the process ID for a ROS node called `talker`.
    ```bash
    ps aux | grep talker
    ```
    This takes the full process list from `ps aux` and "pipes" it to `grep`, which filters it to only show lines containing "talker".

Mastering these commands will make you vastly more effective as a robotics developer, allowing you to quickly diagnose problems, manage files, and interact with the complex software systems running on your robot.
