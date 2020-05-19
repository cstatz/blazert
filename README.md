# blazeRT

1. [Introduction](#introduction)
2. [Features](#features)
3. [Installation](#installation)
    1. [Dependencies](#dependencies)
    2. [Build and Test](#build-and-test)
4. [Usage](#usage)
    1. [Examples](#examples)
    2. [Minimal Examples](#minimal-example)
5. [License](#license)
6. [Contributing](#contributing)

## Introduction
A **double precision ray tracer** for physics applications based on a [nanort](https://github.com/lighttransport/nanort) fork using blaze datatypes. blazeRTs scene interface is similar to embree and intents to be a minimal effort (nearly plugin) replacement. 

You can use your own (vector) dataypes (e.g. as provided by eigen3) and it also works with single precision floats.

At the moment blazeRT works with triangular meshes and simple primitives, but it should be easy to extend blazeRT 
to  work on polygons or more complex primitives.

Please read the [contribution guide](CONTRIBUTING.md) if you are interested in improving blazeRT.

## Features
- [x] modern C++
- [x] using vector and matrix type from [blaze](https://bitbucket.org/blaze-lib/blaze/src/master/) for efficient linear algebra
- [x] single and double precision ray tracing 
- [x] Embree fall back for single precision floats
- [x] currently supported geometry
    - [x] triangular meshes
    - [x] spheres
    - [x] (finite) planes
    - [x] cylinders
- [x] BVH accelerated ray racing
- [x] unit tests via [doctest](https://github.com/onqtam/doctest)

## Installation
Installation and build is tested on linux (e.g. ubuntu bionic, arch-linux) and macos.
Before starting the build process please ensure all dependencies are properly installed and available to the project.

### Dependencies
 * c++17 capable compiler
 * cmake (>= 3.11.0)
 * blaze (>= 3.7)
 * embree (>= 3) if ```EMBREE_TRACING``` fallback is desired
 * doctest for testing

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

**A word of caution:** blazeRT will compile and work with compiler optimizations enabled (up to **-O3**), but needs infinite-math. If your application needs fast-math, ensure that the blazeRT code path is compiled with `-fno-finite-math-only` (in case of clang). In terms of performance, in its current form there is no major runtime difference between compilation with *-O2* and *-O3*. 

## Usage
To get familiar with the usage of blazeRT, look at the provided examples and test cases. To get started quickly,
checkout the minimal examples below.
### Examples
- [x] [path tracerfor mesh geometries](examples/path_tracer) with rendered output
- [x] [path tracer based on the scene facility](examples/scene_mesh) of blazeRT without rendered output
- [x] [cylinder and sphere primitives](examples/scene_primitives) within a blazeRT scene and color-coded output
- [ ] Embree fallback

### Minimal Example
Example for using blazeRT can be found the the `examples` subdirectory.

A minimal example would look like this:
```examples/scene_primitives``` to get a feeling for the API.
```c++
#include <blazert/blazert.h>
#include <memory>
#include <vector>

// This alias is defined in order to setup the simulation for float or double, depending on what you want to do.
using ft = double;

int main(int argc, char **argv) {

  int width = 8192;
  int height = 8192;

  // the properties of the cylinders need to be saved on the heap
  auto centers = std::make_unique<blazert::Vec3rList<ft>>();
  auto semi_axes_a = std::make_unique<std::vector<ft>>();
  auto semi_axes_b = std::make_unique<std::vector<ft>>();
  auto heights = std::make_unique<std::vector<ft>>();
  auto rotations = std::make_unique<blazert::Mat3rList<ft>>();

  blazert::Mat3r<ft> rot{
      {0, 0, 1},
      {0, 1, 0},
      {-1, 0, 0}};

  /***
   * Each cylinder adds an element to the std::vectors containing the corresponding parameters
   */
  // cylinder 1
  centers->emplace_back(blazert::Vec3r<ft>{-3, 3, 0});
  semi_axes_a->emplace_back(1);
  semi_axes_b->emplace_back(1);
  heights->emplace_back(1);
  rotations->push_back(rot);

  // cylinder 2
  centers->emplace_back(blazert::Vec3r<ft>{1, 4, 0});
  semi_axes_a->emplace_back(0.5);
  semi_axes_b->emplace_back(0.5);
  heights->emplace_back(2);
  rotations->push_back(rot);

  // Create the scene, add the cylinders and commit the scene
  // committing the scene builds the BVHs
  // After committing you cannot add anymore geometries
  blazert::Scene<ft> scene;
  scene.add_cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);
  scene.commit();
  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      const blazert::Ray<ft> ray{{0.0, 5.0, 20.0}, {static_cast<ft>((x / ft(width)) - 0.5), static_cast<ft>((y / ft(height)) - 0.5), ft(-1.)}};
      blazert::RayHit<ft> rayhit;

      const bool hit = intersect1(scene, ray, rayhit);
      if (hit) { 
        // Do something ...
      }
    }
  }

  return 0;
}
```
## License

blazeRT is licensed under the new **BSD (3-clause) license**.
blazeRT is based on and inspired by `nanort.h` which is licensed under:

```
MIT License

Copyright (c) 2015-2018 Light Transport Entertainment, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

The purpose of this fork is the simplified inclusion of the blaze-lib vector-types (or similar types that provided the necessary operators and datalayout) and an enhanced maintainability of the code.

The examples are built around third party libraries (e.g. `tiny_obj_loader` and `stb_image_write`) which adhere to their own respective licenses (found in the included files).

The rendering example is take from the `nanort` repo and serves as a baseline. The Lucy model included in this demo scene is taken from the Stanford 3D Scanning Repository: http://graphics.stanford.edu/data/3Dscanrep/

## Contributing
We appreciate all contributions from issues to pull requests.

For contributing, please read the [contribution guide](CONTRIBUTING.md).
