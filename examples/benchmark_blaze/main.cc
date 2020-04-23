#include "tiny_obj_loader.h"
#include <blazert/blazert.h>
#include <blazert/bvh/accel.h>
#include <blazert/datatypes.h>
#include <iostream>
#include <vector>

typedef struct {
  blazert::Vec3rList<double> vertices;
  blazert::Vec3iList triangles;
} Mesh;

bool LoadObj(Mesh &mesh, const char *filename) {

  std::vector<tinyobj::shape_t> shapes;
  std::string err = tinyobj::LoadObj(shapes, filename);

  if (!err.empty()) {
    std::cerr << err << std::endl;
    return false;
  }

  std::cout << "[LoadOBJ] # of shapes in .obj : " << shapes.size() << std::endl;

  size_t vertexIdxOffset = 0;

  for (auto &shape : shapes) {
    for (size_t f = 0; f < shape.mesh.indices.size() / 3; f++) {
      blazert::Vec3i t{unsigned(shape.mesh.indices[3 * f + 0]), unsigned(shape.mesh.indices[3 * f + 1]), unsigned(shape.mesh.indices[3 * f + 2])};
      t += vertexIdxOffset;
      mesh.triangles.push_back(t);
    }

    for (size_t v = 0; v < shape.mesh.positions.size() / 3; v++) {
      blazert::Vec3r<double> vv = {shape.mesh.positions[3 * v + 0], shape.mesh.positions[3 * v + 1], shape.mesh.positions[3 * v + 2]};
      mesh.vertices.push_back(vv);
    }

    vertexIdxOffset += shape.mesh.positions.size() / 3;
  }

  std::cout << mesh.vertices.size() << ":" << mesh.triangles.size() << std::endl;
  return true;
}

int main(int argc, char **argv) {

  int width = 8192;
  int height = 8192;

  std::string objFilename = "../../../examples/common/cornellbox_suzanne_lucy.obj";

  if (argc > 1) {
    objFilename = std::string(argv[1]);
  }

  Mesh *mesh = new Mesh();
  LoadObj(*mesh, objFilename.c_str());

  blazert::BVHBuildOptions<double> build_options;// Use default option
  build_options.cache_bbox = false;

  blazert::TriangleMesh<double> triangle_mesh(mesh->vertices, mesh->triangles);   //, sizeof(float) * 3);
  blazert::TriangleSAHPred<double> triangle_pred(mesh->vertices, mesh->triangles);//, sizeof(float) * 3);

  blazert::BVHAccel<double> accel;
  accel.Build(triangle_mesh, triangle_pred, build_options);

  blazert::BVHBuildStatistics stats = accel.GetStatistics();

  //float3 bmin[3], bmax[3];
  //accel.BoundingBox(bmin, bmax);
  blazert::Ray<double> rd;
  blazert::Ray<float> rf;
  std::cout << sizeof(rd) << ":" << sizeof(rf) << std::endl;

  #ifdef _OPENMP
  #pragma omp parallel for schedule(dynamic, 1)
  #endif
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      blazert::Vec3r<double> rayOrg{0.0, 5.0, 20.0};
      blazert::Vec3r<double> rayDir{(x / width) - 0.5, (y / height) - 0.5, -1.0};
      rayDir = normalize(rayDir);

      blazert::Ray<double> ray(rayOrg, rayDir);
      double kFar = 1.0e+30;
      ray.min_t = 0.001;
      ray.max_t = kFar;
      blazert::TriangleIntersector<double> triangle_intersector(mesh->vertices, mesh->triangles);
      blazert::TriangleIntersection<double> isect;
      bool hit = accel.Traverse(ray, triangle_intersector, isect);
      (hit);
    }
  }

  return 0;
}
