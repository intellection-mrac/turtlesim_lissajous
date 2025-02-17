# Turtlesim - Lissajous Curve

This repository demonstrates how to generate complex motion patterns in ROS Noetic using the turtlesim simulator. The `turtlesim_lissajous` package drives a turtle to trace a Lissajous curve—a parametric path created by the superposition of two orthogonal harmonic oscillations. This guide covers the mathematical foundations, ROS node implementation with a proportional controller, and a Docker-based deployment that ensures reproducibility across environments.

Author	:	**intellection**, o3-mini  
email	:	santosh.shenbagamoorthy@students.iaac.net

---

## Table of Contents

1. [Introduction](#introduction)  
   - Overview of the project and its purpose.
   - Explanation of using **ROS Noetic** and **turtlesim** to draw a **Lissajous curve**.
   - Description of Docker-based deployment for consistent development and execution.

2. [Mathematical Background: Lissajous Curves](#mathematical-background-lissajous-curves)  
   - Mathematical formulation of **Lissajous curves**.
   - Key parameters (**A**, **B**, **a**, **b**, **δ**) and their influence on the shape of the curve.
   - Derivatives of the position functions and their use in determining velocity and angular velocity.
   - Overview of the proportional control law to drive the turtle along the curve.

3. [System Overview](#system-overview)  
   - Breakdown of the components in the project:
     - **ROS Noetic Package** (`turtlesim_lissajous`)
     - **ROS Node** responsible for velocity commands and curve following.
     - **Docker Environment** ensuring consistent setup.
     - **Data Logging** with **rosbag** for capturing simulation data.

4. [Docker-Centric Setup](#docker-centric-setup)  
   - **Dockerfile Explanation**: Detailed breakdown of how Docker is used to encapsulate the ROS Noetic environment and dependencies.
   - **Building the Docker Image**: Steps to build the Docker image using the provided script (`build_image.sh`).
   - **Run-Time Scripts Overview**: Description of scripts (`run_user.sh`, `entrypoint.sh`, `setup.bash`) for running and configuring the Docker container.
   - **Running the Docker Container**: How to launch the container, with necessary environment settings and X11 access for GUI tools.

5. [ROS Package Overview](#ros-package-overview)  
   - Overview of the **`turtlesim_lissajous`** ROS package.
   - **Source Code**: Explanation of the key parts of the `lissajous_drawer.py` Python script responsible for controlling the turtle.
   - How the **Lissajous equations** are implemented in the code, and how the turtle’s trajectory is adjusted using velocity commands.

6. [Execution and Troubleshooting](#execution-and-troubleshooting)  
   - Detailed guide on how to execute the simulation:
     - Start the **Turtlesim** GUI.
     - Run the **Lissajous Drawer Node**.
   - Troubleshooting steps for common issues, such as:
     - **No GUI in Docker**.
     - **No ROS Master** running.
     - **X server** and **permissions** issues.

7. [Data Capture Using Rosbag](#data-capture-using-rosbag)  
   - Instructions for recording ROS topics using **rosbag**.
   - Example commands for capturing single or multiple topics for later analysis.
   - How to stop the recording and save the captured data.

8. [Project Folder Structure](#project-folder-structure)  
   - Overview of the project directory structure and the location of key files:
     - Docker-related files in `.docker/`.
     - ROS package in `/dev_ws/src/turtlesim_lissajous`.
     - Key Python and configuration files.

9. [References](#references)  
   - References to scientific literature and external resources related to Lissajous curves, ROS, and turtlesim.
   - Links to MathWorld and other sources.

10. [License](#license)  

---

## 1. Introduction

This project leverages **ROS Noetic** and the **turtlesim** simulation to visualize a Lissajous curve—a trajectory defined by two sinusoidal functions with different frequencies and a phase offset. A proportional controller ensures that the turtle accurately follows the computed path. The entire system is containerized using Docker, ensuring consistency across various host environments. The provided Docker-based deployment includes helper scripts to build the Docker image, initialize the ROS environment, and launch the container with proper user privileges and X11 access for GUI applications.

---

## 2. Mathematical Background: Lissajous Curves

Lissajous curves, named after **Jules Antoine Lissajous**, are defined by the following parametric equations:

  x(t) = center_x + A · sin(a · t + δ)  
  y(t) = center_y + B · sin(b · t)

**Key Parameters:**

- **A**, **B**: Amplitudes along the x- and y-axes.
- **a**, **b**: Angular frequencies. The ratio a:b determines the number of lobes and intersections.
- **δ**: Phase shift in radians (for example, δ = π/2 for an orthogonal configuration).
- **center_x**, **center_y**: Center coordinates of the curve (typically centered on an 11×11 grid in turtlesim).

### Velocity and Control Derivatives

The first derivatives of the position functions are:

  dx/dt = A · a · cos(a · t + δ)  
  dy/dt = B · b · cos(b · t)

The instantaneous linear speed is given by:

  v = √((dx/dt)² + (dy/dt)²)

And the desired heading angle is determined by:

  θ = arctan2(dy/dt, dx/dt)

A proportional control law then adjusts the angular velocity so that the turtle’s heading aligns with the tangent of the Lissajous curve.

**Scientific References:**

- Lissajous, J. A. (1857). *Mémoire sur l'étude optique des mouvements vibratoires.* Journal de Mathématiques Pures et Appliquées.
- Weisstein, Eric W. "Lissajous Curve." MathWorld – [Wolfram Web Resource](https://mathworld.wolfram.com/LissajousCurve.html).

---

## 3. System Overview

The project consists of the following components:

- **ROS Noetic Package:**  
  The `turtlesim_lissajous` package includes a ROS node that computes and publishes velocity commands to drive the turtle along the Lissajous curve.

- **ROS Node:**  
  Implemented in Python (see [Source Code: lissajous_drawer.py](#source-code-lissajous_drawerpy)). This node calculates the necessary velocity commands based on the Lissajous equations and uses a proportional controller for smooth trajectory tracking.

- **Docker Environment:**  
  A Docker image encapsulates the complete ROS development and runtime environment to ensure consistency. A Catkin workspace is set up at `/dev_ws` along with all necessary ROS dependencies.

- **Data Logging:**  
  Instructions for using rosbag are provided to capture simulation data for troubleshooting or further analysis.

---

## 4. Docker-Centric Setup

Docker is employed to package the entire ROS environment, eliminating potential discrepancies between host systems. All Docker-related files and scripts are located in the `.docker` folder of the repository.

### Docker Integration with ROS

The entire setup is packaged using Docker to ensure consistent development and execution environments. Here's how Docker integrates into the ROS package:

- **Dockerfile:**  
  The Dockerfile builds a container that installs ROS Noetic, along with all dependencies required to run your package. It also installs necessary tools like Python, CMake, and other utilities needed to develop and run ROS nodes.

- **run_user.sh:**  
  This script runs the Docker container with appropriate user permissions and mounts for X11 forwarding, allowing you to interact with GUI tools like `rqt` and `rviz` within the container. It ensures a seamless experience when using ROS tools that require GUI rendering.

- **entrypoint.sh:**  
  This script sources the ROS and workspace setup files to ensure that the ROS environment is correctly initialized when the container starts.

### Dockerfile Explanation

The Dockerfile (located in the `.docker` directory) defines the container’s build process. It installs required dependencies, sets up a Catkin workspace, and compiles the ROS package. Below is an excerpt highlighting the key components:

```dockerfile
ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO-ros-base

ARG ROS_DISTRO

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color

# Install common utilities and development packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ssh \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    python3-pip \
    python3-flake8 \
    terminator \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install additional ROS packages and simulation tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall-generator \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-plugins \
    ros-$ROS_DISTRO-roslint \
    ros-$ROS_DISTRO-rqt-gui \
    ros-$ROS_DISTRO-rqt-gui-py \
    ros-$ROS_DISTRO-rqt-py-common \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-ecl-threads \
    ros-$ROS_DISTRO-ecl-geometry \
    ros-$ROS_DISTRO-ecl-streams \
    ros-$ROS_DISTRO-diagnostics \
    ros-$ROS_DISTRO-turtlesim \
    ros-$ROS_DISTRO-ros-tutorials \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Set up the workspace directory and copy source files
WORKDIR /
RUN mkdir -p dev_ws/src

COPY . /dev_ws/src/

WORKDIR /dev_ws
RUN ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build"]

# Copy entrypoint and ROS setup scripts to the correct locations
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint.sh /entrypoint.sh && chmod 777 /entrypoint.sh"]
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/setup.bash /dev_ws/setup.bash && chmod 777 /dev_ws/setup.bash"]

# Set the default entrypoint and command to launch a bash shell
ENTRYPOINT ["bash", "/entrypoint.sh" ]
CMD ["bash"]
```

### Building the Docker Image

All scripts related to building the image are now located in the `.docker` folder. The helper script `build_image.sh` simplifies the build process. Its contents are as follows:

```bash
#!/usr/bin/env bash
set -e

echo -e "Building ros_introduction:latest image"

DOCKER_BUILDKIT=1 \
docker build --pull --rm -f .docker/Dockerfile \
  --build-arg BUILDKIT_INLINE_CACHE=1 \
  --tag ros_introduction:latest .
```

To build the image, execute this command from the repository root:

```bash
./.docker/build_image.sh
```

Alternatively, build manually with:

```bash
DOCKER_BUILDKIT=1 docker build --pull --rm -f .docker/Dockerfile \
  --build-arg BUILDKIT_INLINE_CACHE=1 --tag ros_introduction:latest .
```

### Run-Time Scripts Overview

The `.docker` directory also contains scripts to run the container and set up the ROS environment:

1. **run_user.sh**  
   This script initializes the environment and launches Docker using appropriate user privileges and X11 access for graphical applications:

   ```bash
   #!/usr/bin/env bash
   set -e
   
   echo -e "Starting up ros_introduction container \n >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
   echo -e "This container will have access to your home directory, and you will log in as your user with proper X11 access."
   echo -e "Note: You may need to adjust the file ownership of the workspace by running:\n  sudo chown -R \$USER /dev_ws"
   echo -e "Source the workspace with: source devel/setup.bash"
   
   # Launch the container with the appropriate mounts and privileges
   docker run -it --privileged \
       --user=$(id -u $USER):$(id -g $USER) \
       --group-add sudo \
       --env="DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --workdir="/dev_ws" \
       --volume="/home/$USER:/home/$USER" \
       --volume="/etc/group:/etc/group:ro" \
       --volume="/etc/passwd:/etc/passwd:ro" \
       --volume="/etc/shadow:/etc/shadow:ro" \
       --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="/dev:/dev" \
       --net=host \
       --cap-add=sys_nice \
       ros_introduction:latest
   ```

2. **entrypoint.sh**  
   This script sets up the ROS environment inside the container. It is invoked as the container's entry point:	

   ```bash
   #!/usr/bin/env bash
   set -e
   
   # Set up the ROS environment
   source "/opt/ros/noetic/setup.bash"
   source "/dev_ws/devel/setup.bash"
   
   exec "$@"
   ```

3. **setup.bash**  
	Copied to `/dev_ws/setup.bash` during the build process, this script simplifies sourcing the ROS environment for the Catkin workspace.
	
	```bash
	source "/opt/ros/noetic/setup.bash"
	source "/dev_ws/devel/setup.bash"
   ```


### Running the Docker Container

After building your image, you can launch the container with the following command from the repository root:

```bash
./.docker/run_user.sh
```

This command launches the container in privileged mode with the required mounts and environment settings. It gives you a shell session with ROS properly sourced (via `/dev_ws/setup.bash` and `/opt/ros/noetic/setup.bash`).

---

## 5. ROS Package Overview

The `turtlesim_lissajous` package (located in `/dev_ws/src/turtlesim_lissajous`) is responsible for commanding the turtle along the Lissajous curve. It publishes velocity commands computed from the parametric equations.

### Source Code: lissajous_drawer.py

The ROS node is implemented in Python and is located at `/dev_ws/src/turtlesim_lissajous/scripts/lissajous_drawer.py`:

```python
#!/usr/bin/env python3
"""
lissajous_drawer.py

This ROS node commands turtlesim to draw a Lissajous curve defined by:
    x(t) = center_x + A * sin(a * t + delta)
    y(t) = center_y + B * sin(b * t)

Derivatives:
    dx/dt = A * a * cos(a * t + delta)
    dy/dt = B * b * cos(b * t)

The instantaneous linear velocity is computed as:
    v = sqrt((dx/dt)^2 + (dy/dt)^2)
and the desired heading is computed using:
    theta = arctan2(dy/dt, dx/dt)

A proportional controller adjusts the angular velocity so that the turtle
matches the tangent direction of the curve.

Author: intellection, o3-mini
References:
    - Lissajous, J. A. (1857). Mémoire sur l'étude optique des mouvements vibratoires.
    - Weisstein, Eric W. "Lissajous Curve", MathWorld.
"""

import rospy
from geometry_msgs.msg import Twist
import math

def lissajous_drawer():
    # Initialize the ROS node
    rospy.init_node('lissajous_drawer', anonymous=True)
    vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(30)  # 30 Hz update frequency for smooth control

    # Lissajous curve parameters
    A = 4.0              # Amplitude along the x-axis
    B = 4.0              # Amplitude along the y-axis
    a = 3.0              # Angular frequency for the x component
    b = 4.0              # Angular frequency for the y component
    delta = math.pi / 2  # Phase shift in radians

    # Duration for drawing (in seconds)
    duration = 30.0

    # Turtlesim grid center (typically an 11x11 grid)
    center_x = 5.5
    center_y = 5.5

    start_time = rospy.get_time()
    vel_msg = Twist()

    while not rospy.is_shutdown():
        t = rospy.get_time() - start_time
        if t > duration:
            break

        # Calculate the derivatives
        dx_dt = A * a * math.cos(a * t + delta)
        dy_dt = B * b * math.cos(b * t)

        # Compute the linear velocity (Euclidean norm)
        linear_velocity = math.sqrt(dx_dt**2 + dy_dt**2)

        # Determine the desired heading using arctan2 (to correctly handle quadrants)
        desired_heading = math.atan2(dy_dt, dx_dt)

        # Proportional controller for angular velocity
        k_angular = 4.0  # Controller gain
        angular_velocity = k_angular * desired_heading

        # Publish velocity commands (adjust linear velocity scaling as needed)
        vel_msg.linear.x = linear_velocity * 0.1
        vel_msg.angular.z = angular_velocity
        vel_pub.publish(vel_msg)

        rate.sleep()

    # Stop the turtle after drawing the curve
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        lissajous_drawer()
    except rospy.ROSInterruptException:
        pass
```

> **Note:** Ensure the Python script is executable:
> 
> ```bash
> chmod +x /dev_ws/src/turtlesim_lissajous/scripts/lissajous_drawer.py
> ```

---
### CMakeLists.txt

The `CMakeLists.txt` file is responsible for managing the build system of the ROS package using CMake. Below is a detailed breakdown:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_lissajous)

# This is the ROS build system configuration
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  turtlesim
)

# Declare a catkin package
catkin_package()

# Include directories (for headers or any shared resources)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python scripts don't require compilation, just installation.
catkin_install_python(PROGRAMS
  scripts/lissajous_drawer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install any necessary directories (for configuration or data files)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

**Explanation:**
	
**cmake_minimum_required(VERSION 3.0.2):**  
Ensures that CMake version 3.0.2 or higher is used for the build process.

**project(turtlesim_lissajous):**  
Names the project `turtlesim_lissajous`, which will be used for logging and identifying the build process.

**find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs turtlesim):** 
This line searches for the necessary ROS packages required for the node:

   - `rospy`: The Python client library for ROS.

   - `geometry_msgs`: A message package containing standard geometry message types like `Twist` (for velocity commands).

   - `turtlesim`: The turtlesim package, which is necessary to simulate the turtle.

**catkin_package():**  
Declares the project as a catkin package, enabling it to work with the Catkin build system.

**include_directories(${catkin_INCLUDE_DIRS}):**  
Specifies the include directories, which are necessary for finding header files from the ROS packages specified in `find_package()`.

**catkin_install_python(PROGRAMS scripts/lissajous_drawer.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}):**  
This installs the Python script `lissajous_drawer.py` into the appropriate binary destination folder, making it executable as a ROS node.

**install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}):**  nstalls the launch directory (if present) into the correct location, allowing for easy execution of launch files.

### package.xml

The `package.xml` file contains metadata about the package, its dependencies, and other essential configuration.

```xml
<?xml version="1.0"?>
<package format="2">
  <name>turtlesim_lissajous</name>
  <version>0.0.0</version>
  <description>
    The turtlesim_lissajous package implements a ROS node to drive the turtle
    through a Lissajous curve using a proportional controller.
  </description>

  <maintainer email="santosh.shenbagamoorthy@students.iaac.net">intellection</maintainer>

  <license>MIT</license>

  <depend>rospy</depend>
  <depend>geometry_msgs</depend>
  <depend>turtlesim</depend>

  <buildtool_depend>catkin</buildtool_depend>

  <exec_depend>ros-noetic-turtlesim</exec_depend>

  <export>
    <build_type>catkin</build_type>
  </export>
</package>
```

---

## 6. Execution and Troubleshooting

If you encounter issues during execution, here's a step-by-step troubleshooting guide:

1. **No GUI in Docker:**  
   Ensure that the `DISPLAY` environment variable is properly set. If you are using an X11 server on your host machine, check the permissions for the `/tmp/.X11-unix` directory and confirm that X11 forwarding is enabled.

2. **No ROS Master:**  
   Make sure that the ROS master is running. You can do this by running `roscore` in a separate terminal.

3. **Container Setup Issues:**  
   If the container doesn't launch properly or doesn't have the necessary permissions, ensure that you are using the correct `run_user.sh` script and that your user has the necessary privileges to access the Docker container.



Follow these steps to run the simulator and resolve any issues:

1. **Start Turtlesim:**

   Open a new terminal within the container and run:
   
   ```bash
   rosrun turtlesim turtlesim_node
   ```
   
   If the GUI does not display, ensure your X server settings are correct and the `DISPLAY` environment variable is properly set.

2. **Run the Lissajous Drawer Node:**

   In another terminal, source the workspace and launch the node:
   
   ```bash
   source /dev_ws/setup.bash
   rosrun turtlesim_lissajous lissajous_drawer.py
   ```
   
   The turtlesim window should display the turtle drawing a Lissajous curve over a 30-second period.

3. **Troubleshooting Tips:**

   - **ROS Master:**  
     Verify that the ROS master node (`roscore`) is running. The container setup might automatically handle this.
   
   - **X Server Issues:**  
     Double-check the file permissions for `/tmp/.X11-unix` and confirm that the `DISPLAY` variable is set correctly.
   
   - **Build Errors:**  
     Ensure that the Catkin workspace builds without errors by reviewing build logs.

---

## 7. Data Capture Using Rosbag

You can record simulation data using **rosbag** for later analysis or debugging:

### Record a Single Topic

```bash
source /dev_ws/setup.bash
rosbag record -O ~/out/lissajous_turtle.bag /turtle1/cmd_vel
```

### Record Multiple Topics

```bash
source /dev_ws/setup.bash
rosbag record -O ~/out/lissajous_data.bag /turtle1/cmd_vel /turtle1/pose
```

Press **CTRL+C** to stop recording once you have captured the desired data.

---

Here is a breakdown of the `CMakeLists.txt` and `package.xml` file, along with a detailed explanation of how each component functions within your ROS Noetic project. This ensures the project is set up and works correctly.

---

## 8. Project Folder Structure

The repository is organized as follows:

```
turtlesim_lissajous/
├── .docker/                   # Docker-related files (build, run, entrypoint scripts)
│   ├── build_image.sh         # Helper script to build the Docker image
│   ├── entrypoint.sh          # ROS environment setup script for Docker container
│   ├── run_user.sh            # Script to launch Docker container with user permissions
│   ├── setup.bash             # Script for workspace setup in the Docker container
│   └── Dockerfile             # Instructions for building the Docker image
├── dev_ws/                    # ROS workspace (Catkin workspace)
│   ├── src/                   # Source code directory for ROS packages
│   │   └── turtlesim_lissajous/
│   │       ├── launch/        # ROS launch files (if applicable)
│   │       ├── scripts/       # ROS Python nodes and scripts
│   │       │   └── lissajous_drawer.py  # Python script to control the turtle
│   │       ├── CMakeLists.txt # CMake build configuration for ROS package
│   │       └── package.xml    # ROS package metadata and dependencies
│   └── build/                 # Directory where ROS builds the packages
├── logs/                      # Directory for storing logs (e.g., rosbag files, outputs)
└── README.md                  # Project documentation (this file)
```

### Key Folders and Files:
- **.docker**: Contains all Docker-related files needed for building, running, and setting up the containerized environment.
- **dev_ws/src**: ROS source code directory, where the main `turtlesim_lissajous` package resides. The Python script `lissajous_drawer.py` is located here.
- **dev_ws/build**: ROS builds the packages in this directory. It's automatically managed by the ROS build system (Catkin).
- **dev_ws/devel**: This is the workspace’s development environment, where compiled ROS packages are stored.
- **logs**: Stores any output from rosbag recordings or other log data captured during the simulation.

---

## 9. References

- **Lissajous Curves**:
  - Lissajous, J. A. (1857). *Mémoire sur l'étude optique des mouvements vibratoires.* Journal de Mathématiques Pures et Appliquées.
  - Weisstein, Eric W. "Lissajous Curve," MathWorld – [Wolfram Web Resource](https://mathworld.wolfram.com/LissajousCurve.html).

- **ROS Documentation**:  
  - ROS Noetic documentation: [ROS Noetic Wiki](http://wiki.ros.org/noetic)

- **Proportional Control**:  
  - Nise, Norman S. *Control Systems Engineering* (2015). Wiley.

---

## 10. License

This project is released under the MIT License. See the `LICENSE` file for more details.

---

This concludes the detailed breakdown of the `turtlesim_lissajous` repository. The project leverages ROS Noetic and the turtlesim simulator to trace a Lissajous curve using a proportional controller, and it’s fully containerized using Docker for reproducibility across different environments.
