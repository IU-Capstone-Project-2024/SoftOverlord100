# SimOverlord100

# Simulator part of Overlord100 Mobile Platform. 
This repository TBA

## Development: Devcontainer
For the sake of development simplicity, DevContainer environment was created to preserve dependencies integrity and allow to avoid damaging system with local installation.

### Usage
1. Install VS Code and Dev Container extension
2. Press `Ctrl+Shift+P` and choose option `DevContainers: Reopen in Container`.
3. Choose the platform you are working on (`arm64` if you are using Apple Sillicon, otherwise `x86`)
4. Run main launch file
```
ros2 launch urdf_dummy gazebo.launch.py
```
> *note:* The Apple Sillicon is criticaly unstable

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




