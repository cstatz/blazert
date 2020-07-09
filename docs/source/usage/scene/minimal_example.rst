Getting Started
===============
The following examples shows all aspects of the high-level scene API. The code we show here is very similar to
``examples/scene_primitives/main.cc``; however, we excluded writing to a PNG file here.

.. note:: The scene API is considered to be stable.

Setup
-------------

Before we can start using blazert we need to include a few headers from the C++ standard library and more importantly
the blazert header itself:

.. code-block:: cpp
    :linenos:

    #include <iostream>
    #include <memory>
    #include <vector>

    // blazert includes
    #include <blazert/blazert.h>

``blazert/blazert.h`` includes all headers necessary to use blazert. If you only need parts or want a more fine-grained
include, you can include single headers as well.

Furthermore, to easily change which floating point type we want to use, we define a type alias.

.. code-block:: cpp
    :linenos:

    // This alias is defined in order to setup the simulation for
    // float or double, depending on what you want to do.
    using ft = double;

We also define the dimensions for a rendering case which is later used for ray generation. We also define a rotation
matrix ``rot`` which we will later use to rotate the cylinders around the y-axis.

.. code-block:: cpp
    :linenos:

    int main(int argc, char **argv) {

      // define width and height of the rendering
      int width = 1080;
      int height = 720;

      // Define a rotation matrix around y-axis
      blazert::Mat3r<ft> rot{
          {0, 0, 1},
          {0, 1, 0},
          {-1, 0, 0}};

      ...

    }

Creating Primitives
--------------------
To do any ray tracing we need to add some primitives to the scene. blazert comes with some pre-defined primitives:

* sphere
* cylinder
* planes

Besides these simple primitives blazert can also trace against triangle meshes which is probably one of the more common
application scenarios, at least for rendering.

For this example, we want to create two cylinders and one sphere.

First, we need to allocate memory on the heap with the pre-defined data types ``blazert::Vec3rList<T>`` and
``blazert::Matr3rList<T>`` for the blaze data types. This is necessary, because otherwise reallocation of memory might
corrupt the pointers/references deep in blazert.

.. code-block:: cpp
    :linenos:

      // the properties of the cylinders need to be saved on the heap
      auto centers = std::make_unique<blazert::Vec3rList<ft>>();
      auto semi_axes_a = std::make_unique<std::vector<ft>>();
      auto semi_axes_b = std::make_unique<std::vector<ft>>();
      auto heights = std::make_unique<std::vector<ft>>();
      auto rotations = std::make_unique<blazert::Mat3rList<ft>>();


.. note:: ``centers`` and ``rotations`` cannot be regular ``std::vector`` types because for blaze provides a custom allocator for aligned data types.

Next, we need to fill these lists with the information about our objects by adding fields to these lists:

.. code-block:: cpp
    :linenos:

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

Therefore, each ``std::vector`` holds $N$ entries if $N$ primitives are have been added to the scene.

The same can be done with other primitives as well. They follow the same scheme. Only the needed parameters to describe
the primitives differ.

.. code-block:: cpp
    :linenos:

      // We do the same for the spheres
      auto sph_centers = std::make_unique<blazert::Vec3rList<ft>>();
      auto radii = std::make_unique<std::vector<ft>>();
      // sphere 1
      sph_centers->emplace_back(blazert::Vec3r<ft>{1, 1, 0});
      radii->emplace_back(0.5);
      sph_centers->emplace_back(blazert::Vec3r<ft>{-2, 10, 0});
      radii->emplace_back(1.5);

Before we can start path tracing we need to create a ``Scene`` object and add the corresponding primitives via the
``add_<primitive>`` functions. More information on the API can be found in the API reference.

.. code-block:: cpp
    :linenos:

      // Create the scene, add the cylinders and spheres
      blazert::Scene<ft> scene;
      scene.add_cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);
      scene.add_spheres(*sph_centers, *radii);

We are almost set up for path tracing now.

Path Tracing
-------------
Before we can do any ray tracing we need to commit the scene. This means, we get the scene ready for ray tracing by
building the BVH acceleration structure:

.. code-block:: cpp
    :linenos:

      // commits the scene -> build two (trivial) BVHs:
      // - one for cylinders
      // - one for spheres
      scene.commit();

.. note:: Each geometry type has it's own BVH.

Now, we are ready to do the ray tracing. In this case we define a ray origin ``(0, 5, 20)`` and the direction of each ray
is determined by the grid of dimension :math:`\mathrm{width} \times \mathrm{height}`.

The actual ray tracing is done by ``intersect1`` which traverses the BVHs of each geometry type and finds the closest intersection
between the ray and any geometry present in the scene.

.. code-block:: cpp
    :linenos:

      // iterate over the pixels which define the direction of the rays which are launched
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          // create a ray
          const blazert::Ray<ft> ray{
                            {0.0, 5.0, 20.0},
                            {static_cast<ft>((x / ft(width)) - 0.5),
                            static_cast<ft>((y / ft(height)) - 0.5), ft(-1.)}
                        };
          blazert::RayHit<ft> rayhit;

          const bool hit = intersect1(scene, ray, rayhit);
          if (hit) {
            // Do something ...
          }
        }
      }

You are now ready to dive into the blazert framework and use it however you like to.

Complete Code
-------------

The complete code is shown here:

.. code-block:: cpp
    :linenos:

    #include <iostream>
    #include <memory>
    #include <vector>
    #include <blazert/blazert.h>
    #include <blazert/datatypes.h>

    // This alias is defined in order to setup the simulation for
    // float or double, depending on what you want to do.
    using ft = double;

    int main(int argc, char **argv) {

      // define width and height of the rendering
      int width = 1080;
      int height = 720;

      // the properties of the cylinders need to be saved on the heap
      auto centers = std::make_unique<blazert::Vec3rList<ft>>();
      auto semi_axes_a = std::make_unique<std::vector<ft>>();
      auto semi_axes_b = std::make_unique<std::vector<ft>>();
      auto heights = std::make_unique<std::vector<ft>>();
      auto rotations = std::make_unique<blazert::Mat3rList<ft>>();

      // Define a rotation matrix around y-axis
      blazert::Mat3r<ft> rot{
          {0, 0, 1},
          {0, 1, 0},
          {-1, 0, 0}};

      // Each cylinder adds an element to the std::vectors containing the
      // corresponding parameters

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

      // We do the same for the spheres
      auto sph_centers = std::make_unique<blazert::Vec3rList<ft>>();
      auto radii = std::make_unique<std::vector<ft>>();
      // sphere 1
      sph_centers->emplace_back(blazert::Vec3r<ft>{1, 1, 0});
      radii->emplace_back(0.5);
      sph_centers->emplace_back(blazert::Vec3r<ft>{-2, 10, 0});
      radii->emplace_back(1.5);

      // Create the scene, add the cylinders and spheres
      blazert::Scene<ft> scene;
      scene.add_cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);
      scene.add_spheres(*sph_centers, *radii);

      // commits the scene -> build two (trivial) BVHs:
      // - one for cylinders
      // - one for spheres
      scene.commit();

      // iterate over the pixels which define the direction of the rays which are launched
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          // create a ray
          const blazert::Ray<ft> ray{
                            {0.0, 5.0, 20.0},
                            {static_cast<ft>((x / ft(width)) - 0.5),
                            static_cast<ft>((y / ft(height)) - 0.5), ft(-1.)}
                        };
          blazert::RayHit<ft> rayhit;

          const bool hit = intersect1(scene, ray, rayhit);
          if (hit) {
            // Do something ...
          }
        }
      }
      return 0;
    }
