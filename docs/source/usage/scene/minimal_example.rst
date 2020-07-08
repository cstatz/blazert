Minimal Example
===============
The following examples shows all aspects of the high-level scene API.

.. note:: The scene API is considered to be stable.

Includes
-------------


Creating Primitives
--------------------



Complete Code
-------------

.. code-block:: cpp
    :linenos:

    #include <blazert/blazert.h>
    #include <blazert/datatypes.h>
    #include <iostream>
    #include <memory>
    #include <vector>

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
