# Immediate-Mode Toolkit (imtoolkit)

![GitHub Workflow Status](https://github.com/rxdu/imtoolkit/workflows/CMake/badge.svg)

This repository constains a set of packages built on top of the [imgui](https://github.com/ocornut/imgui) library. The targeted use case is creating data visualization and basic UI for assisting development and debugging of robotics software. This toolkit is meant to be very lightweight (minimal dependent libraries required, easy to be integrated and built), which differentiates it from other more feature-rich but much heavier libraries (such as OpenCV, VTK, ROS/RViz). Currently the following features are supported:

* Plot: ImGui + ImPlot
* CairoCanvas: ImGui + Cairo 2D graphics library

## Build 

The code in this repository should build on any recent linux distributions with a compiler supporting C++11/14.

**Setup toolchain**

```
$ sudo apt install build-essential cmake
```

If the version of cmake bundled with your system is too low, you can install a newer version from [kitware ppa](https://apt.kitware.com/) or build & install from [source](https://cmake.org/download/). 

**Install dependencies**

```
$ sudo apt-get install libgl1-mesa-dev libglfw3-dev libcairo2-dev
```

**Download code**

```
$ git clone --recursive https://github.com/rxdu/imtoolkit.git
```

Or you can clone and then update the submodules manually

```
$ git clone https://github.com/rxdu/imtoolkit.git
$ cd imtoolkit
$ git submodule update --init --recursive
```

**Configure and compile**

```
$ cd imtookit
$ mkdir build && cd build
$ cmake ..
$ make -j8
```
