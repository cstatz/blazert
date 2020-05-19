#include "OriginSphere.h"

// OriginSphere::OriginSphere() {}

OriginSphere::OriginSphere(uint32_t resolution) {
  std::vector<OriginMesh> meshes;

  // for (auto &m : meshes) m.clear();

  meshes.emplace_back(OriginMesh());
  initialize_icosahedron(meshes.back());

  for (uint32_t i = 0; i < resolution; i++) {
    meshes.emplace_back(OriginMesh());
  }

  for (uint32_t k = 0; k < resolution; ++k) {

    meshes[k + 1].reserve(20 * std::pow(4, resolution) - 8, 20 * std::pow(4, resolution) - 8);
    subdivide_mesh(meshes[k], meshes[k + 1]);
  }

  triangles = meshes.back().triangles;
  vertices = meshes.back().vertices;
};
