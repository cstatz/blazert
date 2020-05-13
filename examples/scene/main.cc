#include "tiny_obj_loader.h"
#include <blazert/blazert.h>
#include <iostream>
#include <vector>

using ft = float;

template<typename T>
struct Mesh {
  blazert::Vec3rList<T> vertices;
  blazert::Vec3iList triangles;
};

template<typename T>
bool LoadObj(Mesh<T> &mesh, const char *filename) {

  std::vector<tinyobj::shape_t> shapes;
  std::string err = tinyobj::LoadObj(shapes, filename);

  if (!err.empty()) {
    std::cerr << err << std::endl;
    return false;
  }

  size_t vertexIdxOffset = 0;

  for (auto &shape : shapes) {
    for (size_t f = 0; f < shape.mesh.indices.size() / 3; f++) {
      blazert::Vec3ui t{unsigned(shape.mesh.indices[3 * f + 0]), unsigned(shape.mesh.indices[3 * f + 1]), unsigned(shape.mesh.indices[3 * f + 2])};
      t += vertexIdxOffset;
      mesh.triangles.push_back(t);
    }

    for (size_t v = 0; v < shape.mesh.positions.size() / 3; v++) {
      blazert::Vec3r<T> vv = {T(shape.mesh.positions[3 * v + 0]), T(shape.mesh.positions[3 * v + 1]), T(shape.mesh.positions[3 * v + 2])};
      mesh.vertices.push_back(vv);
    }

    vertexIdxOffset += shape.mesh.positions.size() / 3;
  }

  return true;
}

int main(int argc, char **argv) {

  int width = 8192;
  int height = 8192;

  std::string objFilename = "../../../examples/common/cornellbox_suzanne_lucy.obj";

  if (argc > 1) {
    objFilename = std::string(argv[1]);
  }

  // The mesh needs to be on the heap
  auto mesh = new Mesh<ft>();
  LoadObj(*mesh, objFilename.c_str());

  blazert::Scene<ft> scene;
  unsigned int prim_id = scene.add_mesh(mesh->vertices, mesh->triangles);
  scene.commit();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      const blazert::Ray<ft> ray{{0.0, 5.0, 20.0}, {static_cast<ft>((x / ft(width)) - 0.5), static_cast<ft>((y / ft(height)) - 0.5), ft(-1.)}};
      blazert::RayHit<ft> rayhit;

      const bool hit = intersect1(scene, ray, rayhit);
    }
  }

  return 0;
}
