# Immediate Mode UI Toolkit (imtoolkit)

![GitHub Workflow Status](https://github.com/rxdu/imtoolkit/workflows/CMake/badge.svg)

This repository contains the following libraries:

* imgui
* implot

and the main purpose is to provide a CMake configuration so that the two libraries can be easily intergrated into a larger project.

## Build 

Install dependencies

```
$ sudo apt-get install libgl1-mesa-dev libglfw3-dev
```

Configure and compile

```
$ mkdir build && cd build
$ cmake ..
$ make -j8
```