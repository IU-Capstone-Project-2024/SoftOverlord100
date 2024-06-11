# SimOverlord100

# Software part of Overlord100 Mobile Platform. 
This repository assembles different ROS packages to implement a backend part of Overlord100 Mobile Platform. This includes:
1. Controller
2. SLAM
3. Path planner

## Dependencies
The backend is written using ROS2 Humble middleware with the following external dependencies:
   1. [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox) 
   2. [Nav2](https://github.com/ros-navigation/navigation2) 
   3. [ROS2 Control](https://github.com/ros-controls/ros2_control)

They could be installed at:
```bash
RUN apt-get install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-ros2-controllers ros-rolling-ros2-control
```

## Development: Devcontainer
For the sake of development simplicity, DevContainer environment was created to preserve dependencies integrity and allow to avoid damaging system with local installation.

### Usage
1. Install VS Code and Dev Container extension
2. Press `Ctrl+Shift+P` and choose option `DevContainers: Reopen in Container`.
3. Choose the platform you are working on (`amd64` if you are using Apple Sillicon, otherwise `x86`)

### Features
1. Different processors architectures are supported (yet, `x86` works much better)
2. Code linting for `Python` and `C++`


## Development: Continuous Integration
This repository has the following pipelines:
1. Python `black` linting.
2. (wip) C++ `clang-tidy` linting.
3. (wip) ROS2 package build test.

## Development: Code Style
1. `CPP`
   1. `Google` style: [link](https://google.github.io/styleguide/cppguide.html)
   2. `C++ Core Guidelines` style: [link](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)
2. `Python`
   1. `PEP8` style: [link](https://peps.python.org/pep-0008/)
3. `Git Flow`
   1. `Conventional Commits`: [link](https://www.conventionalcommits.org/en/v1.0.0/)




