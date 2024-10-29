# QuickViz

![GitHub Workflow Status](https://github.com/rxdu/quickviz/workflows/default/badge.svg)

This repository provides a C++ framework for creating data visualization and basic GUI for robotics applications. The
core of the framework is a library named "imview". imview is designed to be flexible and lightweight.

* For data visualization, imview provides a set of API functions to easily plot 2D time-series data, draw 2D primitives
  and render 3D objects. It can be used to visualize data in real-time.
* For GUI applications, imview provides automatic layout management and commonly used UI widgets such as buttons,
  sliders, and text boxes.

An app named "quickviz" is provided with commonly used data visualization functions (to support development
of [libxmotion](https://github.com/rxdu/libxmotion)). It also serves as an example of how to use the imview library.
Design of imview is documented in [docs/imview_design.md](docs/imview_design.md). If you are interested in using the
imview library in your own project, it's recommended to read this design document first.

## Build

The code in this repository should build on any recent linux distributions with a compiler supporting C++11/14.

**Setup toolchain**

```
$ sudo apt install build-essential cmake
```

If the version of cmake bundled with your system is too low, you can install a newer version
from [Kitware PPA](https://apt.kitware.com/) or build and install from [source](https://cmake.org/download/).

**Install dependencies**

```
$ sudo apt-get install libgl1-mesa-dev libglfw3-dev libcairo2-dev
```

Please refer to [CI configuration](.github/workflows/default.yml) for the up-to-date dependency installation
instructions.

**Download code**

```
$ git clone --recursive https://github.com/rxdu/quickviz.git
```

Or you can clone and then update the submodules manually

```
$ git clone https://github.com/rxdu/quickviz.git
$ cd quickviz
$ git submodule update --init --recursive
```

**Configure CMake and compile**

```
$ cd quickviz
$ mkdir build && cd build
$ cmake ..
$ make -j8
```

## Reference

The library is built on top of a few third-party libraries, you can refer to their documentation for more details:

* imgui demo: src/third_party/imcore/imgui/imgui_demo.cpp
* implot demo: src/third_party/imcore/implot/implot_demo.cpp
* cairo docs: https://cairographics.org/documentation/
* yoga layout docs: https://www.yogalayout.dev/docs/styling/

Online demo of imgui and implot:

* [imgui demo](https://greggman.github.io/doodles/glfw-imgui/out/glfw-imgui.html)
* [implot demo](https://traineq.org/implot_demo/src/implot_demo.html)
