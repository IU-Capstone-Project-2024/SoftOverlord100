<h1 align="center">
   Overlord100 Simulation
</h1>

<p align="center">
  ROS 2 packages for simulation Autonomus Mobile Platform Overlord100.<br>
</p>


This repository allows the software development team to test the developed algorithms without the risk of damaging the real equipment, as well as to conduct tests in parallel and without the need for personal presence in the laboratory.

# Installation
For the sake of development simplicity, DevContainer environment was created to preserve dependencies integrity and allow to avoid damaging system with local installation.

# Prerequisites

 - [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/) or [Ubuntu 20.04 (Focal Fossa)](https://releases.ubuntu.com/20.04/) or [Windows 11](https://www.microsoft.com/en-gb/software-download/windows11) or [Windows 10](https://www.microsoft.com/en-gb/software-download/windows10ISO) or [MacOS ](https://support.apple.com/en-us/102662)
 - [Docker Desktop](https://www.docker.com/products/docker-desktop/)
 - [VS Code](https://code.visualstudio.com/)
 - [Dev Containers extension for VS Code](https://code.visualstudio.com/docs/devcontainers/containers#_installation)
 - [Git](https://git-scm.com/downloads)
 - [X Server or VcXsrv](https://sourceforge.net/projects/vcxsrv/) (Only for Windows)

> *note:* The Apple Sillicon architecture is criticaly unstable and the preferred OS is Ubuntu


## Build and Launch container through terminal

   - For Linux and MacOS
      
      Firstly, you need to choose, would you download container or build it on your own machine.

      ### Download Container from Harbor Registry or Docker Hub
   
         docker pull harbor.pg.innopolis.university/sim_overlord100/simoverlord:latest
         
      Or

         docker pull simoverlord100/simoverlord:latest

      ### Build on your own machine

      1. Clone the repository to your workspace folder
      ```   
      git clone https://gitlab.pg.innopolis.university/e.shlomov/simoverlord100.git
      ```
      2. Build container
      ```
      docker build -t <name_of_image> .
      ```

      ### Run container
      ```
      docker run -it \
         -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
         -v /dev/dri:/dev/dri:rw \
         <name_of_image> bash
      ```

      ### Setup workspace

      1. Source ROS2 workspace
      ```
      source /opt/ros/humble/setup.bash
      ```
      2. Build ROS2 package
      ```
      cd /home/ws/src
      colcon build
      ```
      
   - For Windows (10 or 11)

      1. Clone the repository to your workspace folder
      ```
      git clone https://gitlab.pg.innopolis.university/e.shlomov/simoverlord100.git
      ```
      2. Add the following line to your Dockerfile (in .devcontainer folder):
      ```
      ENV DISPLAY=host.docker.internal:0.0
      ```
      3. Launch XLaunch app. Left settings as is.
      4. Launch Docker Desktop app
      5. Open cmd.exe in .devcontainer folder of the project folder and execute: 
      ```
      docker build -t <name_of_image> .
      ```
      6. Run Container
      ```
      docker run -it -v X:/<path_to_repository_on_computer>:/develop <name_of_image> bash
      ```

      7. Source ROS workspace in container
      ```
      source /opt/ros/humble/setup.bash
      ```
      8. Build package
      ```
      cd /develop/test_software_overlord100
      sudo colcon build
      ```

## Launch container through Visual Studio Code

   - For MacOS and Linux

      1. Open Visual Studio Code
      2. Press `Ctrl + Shift + G` and choose option `Clone Repository`
      3. Paste repository link
      ```
      https://gitlab.pg.innopolis.university/e.shlomov/simoverlord100.git
      ```   
      4. Select the repository download path
      5. Press `Ctrl+Shift+P` and choose option `DevContainers: Reopen in Container`.
      6. Choose the platform you are working on (`arm64` if you are using Apple Sillicon, otherwise `x86`)
      7. Source ROS 2 workspace
      ```
      source /opt/ros/humble/setup.bash
      ```
      8. Build package
      ```
      cd /home/ws/src
      colcon build
      ```   

## Launch container in local ROS2 workspace 


1. Verify your settings:
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
2. Now add the ROS 2 GPG key with apt.
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
3. Then add the repository to your sources list.
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
4. Update your system
```
sudo apt update

sudo apt upgrade
```
5. Install ROS 2
```
sudo apt install ros-humble-desktop
```
6. Source ROS 2 environment
```
source /opt/ros/humble/setup.bash
```
7. Create new directory
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
8. Clone repo
```
git clone https://gitlab.pg.innopolis.university/e.shlomov/simoverlord100.git
```
9. Resolve dependencies
```
# cd if you're still in the ``src`` directory with the ``simoverlord100`` clone

cd .. #for go to previous directory
rosdep install -i --from-path src --rosdistro humble -y
```
10. Download Gazebo
```
sudo apt-get install ros-humble-ros-gz
sudo apt install ros-humble-rqt-robot-steering
```
11. Download Colcon
```
sudo apt install python3-colcon-common-extensions
```
12. Build workspace with Colcon
```
colcon build
```
13. Source the overlay
```
source /opt/ros/humble/setup.bash
```

Now you can move on to [Starting simultion](#starting-simulation)

## Starting simulation

To start simulation, open terminal and run `ros2 launch` command:
```
source install/local_setup.bash
ros2 launch urdf_dummy <name_of_program>
```

You can choose 2 launch files: `sim_custom_controller.launch.py` and `sim_default_controller.launch.py`.

The `sim_custom_controller.launch.py` is Python launch script that starts simulation with custom robot controller.

The `sim_default_controller.launch.py` is Python launch script that start simulation with predefined simple robot controller.

After that, you will have several windows open. 

In the `RViz` window, the readings of sensors and other systems are visualized.

In the `rqt_robot_steering` window (for launching `sim_default_controller.launch.py` only), you can control the robot by setting the speed and direction of movement. 

To display the simulation window, you need to close the `Gazebo` window, after which a new one will open, with a scene.

## Unit testing

To launch unit testing, open terminal and run `launch_test` command:
```
source install/local_setup.bash
launch_test urdf_dummy/test/test_converter_is_available.py
```

These unit tests checks connection to each sensor that installed on robot.

## Features
1. Different processors architectures are supported (yet, `x86` works much better)
2. Code linting for `Python`

## Development: Continuous Integration
This repository has the following pipelines:
1. Python `black` linting.
2. (wip) ROS2 package build test.
3. Container deployment on Docker Hub and Harbor Registry.
4. Unit testing.

## Development: Code Style
1. `CPP`
   1. `Google` style: [link](https://google.github.io/styleguide/cppguide.html)
   2. `C++ Core Guidelines` style: [link](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)
2. `Python`
   1. `PEP8` style: [link](https://peps.python.org/pep-0008/)
3. `Git Flow`
   1. `Conventional Commits`: [link](https://www.conventionalcommits.org/en/v1.0.0/)

