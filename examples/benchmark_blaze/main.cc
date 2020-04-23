#include <algorithm>
#include <cassert>
#include <climits>
#include <cmath>
#include <ctime>
#include <iostream>
#include <vector>
#include <blazert/blazert_real.h>
#include <blazert/blazert.h>
#include "tiny_obj_loader.h"

using double3 = real3<double>;
using int3 = real3<int>;

typedef struct {
   std::vector<double3> vertices;
   std::vector<int3> triangles;
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

  for (size_t i = 0; i < shapes.size(); i++) {
    for (size_t f = 0; f < shapes[i].mesh.indices.size() / 3; f++) {
      int3 t{int(shapes[i].mesh.indices[3 * f + 0]), int(shapes[i].mesh.indices[3 * f + 1]), int(shapes[i].mesh.indices[3 * f + 2])};
      t += vertexIdxOffset;
      mesh.triangles.push_back(t);
    }

    for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
      double3 vv = {shapes[i].mesh.positions[3 * v + 0], shapes[i].mesh.positions[3 * v + 1], shapes[i].mesh.positions[3 * v + 2]};
      mesh.vertices.push_back(vv);
    }

    vertexIdxOffset += shapes[i].mesh.positions.size() / 3;
  }

  std::cout << mesh.vertices.size() << ":" << mesh.triangles.size() << std::endl; 
  return true;
}


int main(int argc, char **argv) {

  int width = 8192;
  int height = 8192;


  std::string objFilename = "../common/cornellbox_suzanne_lucy.obj";

  if (argc > 1) {
    objFilename = std::string(argv[1]);
  }

  Mesh mesh;
  LoadObj(mesh, objFilename.c_str());

  blazert::BVHBuildOptions<T> build_options;  // Use default option
  //build_options.cache_bbox = false;

  //blazert::TriangleMesh<Vec3f, Vec3i> triangle_mesh(vertices, faces); //, sizeof(float) * 3);
  //blazert::TriangleSAHPred<Vec3f> triangle_pred(vertices, faces); //, sizeof(float) * 3);

  //blazert::BVHAccel<Vec3f> accel;
  //ret = accel.Build(triangle_mesh, triangle_pred, build_options);

  //blazert::BVHBuildStatistics stats = accel.GetStatistics();

  //float3 bmin[3], bmax[3];
  //accel.BoundingBox(bmin, bmax);

//#ifdef _OPENMP
//#pragma omp parallel for schedule(dynamic, 1)
//#endif
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

        double3 rayOrg{0.0, 5.0, 20.0};
        double3 rayDir{(x / width) - 0.5, (y / height) - 0.5, -1.0};
        rayDir = normalize(rayDir);

        blazert::Ray<double> ray;
        double kFar = 1.0e+30;
        ray.min_t = 0.001;
        ray.max_t = kFar;

           
        ray.dir[0] = rayDir[0];
        ray.dir[1] = rayDir[1];
        ray.dir[2] = rayDir[2];
        ray.org[0] = rayOrg[0];
        ray.org[1] = rayOrg[1];
        ray.org[2] = rayOrg[2];

        //blazert::TriangleIntersector<> triangle_intersector(vertices, faces, sizeof(float) * 3);
        //blazert::TriangleIntersection<> isect;
        //bool hit = accel.Traverse(ray, triangle_intersector, &isect);

    }
  }

  return 0;
}
