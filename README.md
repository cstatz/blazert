# blazeRT

1. [Introduction](#introduction)
2. [Features](#features)
3. [Installation](#installation)
    1. [Dependencies](#dependencies)
    2. [Build and Test](#build-and-test)
4. [Usage](#usage)
5. [Contributing](#contributing)

## Introduction
A double precision raytracer for physics applications based on a nanort fork using blaze datatypes.
You can use your own (vector) dataypes (e.g. as provided by eigen3) and it also works in single precision.
At the moment blazeRT works with triangular meshes and simple primitives, but it should be easy to extend blazeRT to  work on polygons or more complex primitives.

## Features
- [x] single and double precision ray tracing 
- [x] Embree fall back for single precision floats
- [x] currently supported geometry
    - [x] triangular meshes
    - [x] spheres
    - [x] (finite) planes
    - [x] cylinders
- [x] BVH accelerated ray racing
- [x] unit tests

## Installation
Installation and build is tested on linux (e.g. ubuntu bionic, arch-linux) and macos.
Before starting the build process please ensure all dependencies are properly installed and available to the project.

### Dependencies
 * c++17 capable compiler
 * cmake (>3.11.0)
 * blaze (>3.7)
 * embree (>3) if ```EMBREE_TRACING``` fallback is desired

### Build and test
This is a header-only library. No need to build anything. Just drop it in your source directory and off you go.
The build step is for the examples.
We strictly recommend an out-of-source build in a separate directory (here for simplicity ```build```) 
Starting in the source directory to project is build from the commandline as follows:
```shell script
mkdir build
cd build 
cmake ../
cmake --build .
cmake --build . -- install  # If package needs to be installed 
ctest  # Runs the tests
```
## Usage
For now, look at the examples and test cases.

## Contributing
We appreciate all contributions from issues to pull requests.

For contributing, please read the [contribution guide][contribution].
                                                     
[contribution]: CONTRIBUTING.md
