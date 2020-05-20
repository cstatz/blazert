#include <blazert/blazert.h>
#include <blazert/datatypes.h>
#include <iostream>
#include <memory>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../common/stb_image_write.h"

// This alias is defined in order to setup the simulation for float or double, depending on what you want to do.
using ft = double;

// TODO: This eats float ... we need to convert.
void SaveImagePNG(const char *filename, const float *rgb, int width,
                  int height) {
  auto *bytes = new unsigned char[width * height * 3];
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      const int index = y * width + x;
      bytes[index * 3 + 0] = (unsigned char) std::max(
          0.0f, std::min(rgb[index * 3 + 0] * 255.0f, 255.0f));
      bytes[index * 3 + 1] = (unsigned char) std::max(
          0.0f, std::min(rgb[index * 3 + 1] * 255.0f, 255.0f));
      bytes[index * 3 + 2] = (unsigned char) std::max(
          0.0f, std::min(rgb[index * 3 + 2] * 255.0f, 255.0f));
    }
  }
  stbi_write_png(filename, width, height, 3, bytes, width * 3);
  delete[] bytes;
}

int main(int argc, char **argv) {

  int width = 8192;
  int height = 8192;

  // the properties of the cylinders need to be saved on the heap
  auto centers = std::make_unique<blazert::Vec3rList<ft>>();
  auto semi_axes_a = std::make_unique<std::vector<ft>>();
  auto semi_axes_b = std::make_unique<std::vector<ft>>();
  auto heights = std::make_unique<std::vector<ft>>();
  auto rotations = std::make_unique<blazert::Mat3rList<ft>>();

  auto sph_centers = std::make_unique<blazert::Vec3rList<ft>>();
  auto radii = std::make_unique<std::vector<ft>>();

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

  // first sphere
  sph_centers->emplace_back(blazert::Vec3r<ft>{1, 1, 0});
  radii->emplace_back(0.5);
  sph_centers->emplace_back(blazert::Vec3r<ft>{-2, 10, 0});
  radii->emplace_back(1.5);

  // Create the scene, add the cylinders and commit the scene
  // committing the scene builds the BVHs
  // After committing you cannot add anymore geometries
  blazert::Scene<ft> scene;
  scene.add_cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);
  scene.add_spheres(*sph_centers, *radii);
  scene.commit();

  // structure to save the colors
  std::vector<float> rgb(width * height * 3, 0.0f);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      const blazert::Ray<ft> ray{{0.0, 5.0, 20.0}, {static_cast<ft>((x / ft(width)) - 0.5), static_cast<ft>((y / ft(height)) - 0.5), ft(-1.)}};
      blazert::RayHit<ft> rayhit;

      const bool hit = intersect1(scene, ray, rayhit);
      if (hit) {
        if (rayhit.prim_id == 0) {
          // red for first prim_id
          rgb[3 * ((height - y - 1) * width + x) + 0] = float(1);
          rgb[3 * ((height - y - 1) * width + x) + 1] = float(0);
          rgb[3 * ((height - y - 1) * width + x) + 2] = float(0);
        } else {
          // green for all other prim_ids
          rgb[3 * ((height - y - 1) * width + x) + 0] = float(0);
          rgb[3 * ((height - y - 1) * width + x) + 1] = float(1);
          rgb[3 * ((height - y - 1) * width + x) + 2] = float(0);
        }
      }
    }
  }

  SaveImagePNG("render.png", &rgb.at(0), width, height);

  return 0;
}
