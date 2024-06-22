<h1 align="center">
   Overlord100 Simulation
</h1>

<p align="center">
  ROS 2 packages for simulation Autonomus Mobile Platform Overlord100.<br>
</p>


This repository allows the software development team to test the developed algorithms without the risk of damaging the real equipment, as well as to conduct tests in parallel and without the need for personal presence in the laboratory.

## Installation
For the sake of development simplicity, DevContainer environment was created to preserve dependencies integrity and allow to avoid damaging system with local installation.

### Prerequisites

 - [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/) or [Windows 11 Pro](https://www.microsoft.com/en-gb/software-download/windows11) or [Windows 10 Pro](https://www.microsoft.com/en-gb/software-download/windows10ISO) or [MacOS ](https://support.apple.com/en-us/102662)
 - [Visual Studio Code](https://code.visualstudio.com/)
 - [Docker Desktop](https://www.docker.com/products/docker-desktop/)
 - [Git](https://git-scm.com/downloads)
 - [X Server or VcXsrv](https://sourceforge.net/projects/vcxsrv/) (Only for Windows)

> *note:* The Apple Sillicon architecture is criticaly unstable and the preferred OS is Ubuntu


### Build the packages

#### For MacOS and Ubuntu

1. Clone the repository to your workspace folder
```
git clone https://gitlab.pg.innopolis.university/e.shlomov/simoverlord100.git
```
2. Launch Docker Desktop app
3. Open cloned repository in VS Code
4. Press `Ctrl+Shift+P` and choose option `DevContainers: Reopen in Container`.
5. Choose the platform you are working on (`arm64` if you are using Apple Sillicon, otherwise `x86`)
6. Source ROS 2 workspace
```
source opt/ros/humble/setup.bash
```
7. Build package
```
cd <path_to_package>
colcon build
```

#### For Windows 10/11 Pro

1. Clone the repository to your workspace folder
```
git clone https://gitlab.pg.innopolis.university/e.shlomov/simoverlord100.git
```
2. Launch Docker Desktop app
3. Build image
```
docker build -t <name_of_image> .
```
4. Run Container
```
docker run -it X:/<path_to_repository_on_computer>:/develop <name_of_image> bash
```
5. Launch XLaunch app
6. Source ROS workspace in container
```
source opt/ros/humble/setup.bash
```
7. Build package
```
cd develop/simoverlord100/test_software_overlord100/
colcon build
```



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




