#pragma once
#ifndef BLAZERT_BLAZERT_SCENE_H
#define BLAZERT_BLAZERT_SCENE_H

#include <blazert/bvh/accel.h>
#include <blazert/bvh/options.h>
#include <blazert/datatypes.h>

#include <blazert/primitives/sphere.h>
#include <blazert/primitives/trimesh.h>

#include <blazert/ray.h>

namespace blazert {

template<typename T>
class BlazertScene {
public:
  BVHBuildOptions<T> build_options;
  BVHTraceOptions<T> trace_options;

  // A lot of private data ...
  mutable bool has_been_committed = false;
  mutable bool has_mesh = false;
  mutable bool has_sphere = false;
  mutable bool has_plane = false;
  mutable bool has_box = false;
  mutable bool has_cylinder = false;
  mutable unsigned int primitives = 0;

  TriangleMesh<T> triangles_;
  TriangleSAHPred<T> triangles_sah_;

  Sphere<T> spheres_;                 // default constructed
  SphereSAHPred<T> spheres_sah_;// default constructed

  BVH<T> bvh_mesh;
  BVH<T> bvh_sphere;
  //BVHAccel<T> plane_accel;
  //BVHAccel<T> box_accel;
  //BVHAccel<T> cylinder_accel;

public:
  BlazertScene() = default;

  // TODO: default arguments ...
  unsigned int add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles);
  //inline unsigned int add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles, Vec3rList &vertex_normals);
  inline unsigned int add_spheres(const std::vector<Vec3r<T>> &centers, const std::vector<T> &radiuss);
  //inline unsigned int add_plane(const Vec3r<T> &origin, const Vec3r<T> &bmin, const Vec3r<T> &bmax, const Matrix3r<T> &rotation);
  //inline unsigned int add_box(const Vec3r<T> &center, const Vec3r<T> &size, const Matrix3r<T> &rotation);

  //template<class X, ...> add_custom_primitive( ... );

  bool commit() {

    // Build all the BVH ...
    if (has_mesh) {
      bvh_mesh.build(triangles_, triangles_sah_, build_options);
    }

    if (has_sphere) {
      bvh_sphere.build(spheres_, spheres_sah_, build_options);
    }

    //place_accel.Build()
    //box_accel.Build()
    has_been_committed = true;
    return has_been_committed;
  };
};

// TODO: Performance critical code should not be a member function (hidden pointer *this), since the compiler will not know how to optimize.
template<typename T>
inline bool intersect1(const BlazertScene<T> &scene, const Ray<T> &ray, RayHit<T> &rayhit) {

  // This may not be optimal, but the interface is simple and everything can (and will) be in-lined.
  RayHit<T> temp_rayhit;
  bool hit = false;

  // Do the traversal for all primitives ...
  if (scene.has_mesh) {
    TriangleIntersector<T> triangle_intersector{*(scene.triangles_.vertices_), *(scene.triangles_.faces_)};
    // TODO: Performance critical code should not be a member function (hidden pointer *this), since the compiler will not know how to optimize.
    const bool hit_mesh = traverse(scene.bvh_mesh, ray, triangle_intersector, temp_rayhit, scene.trace_options);
    if (hit_mesh) {
      rayhit = temp_rayhit;
      hit += hit_mesh;
    }
  }

  if (scene.has_sphere) {
    SphereIntersector<T> sphere_intersector{*(scene.spheres_.centers_), *(scene.spheres_.radiuss_)};
    const bool hit_sphere = traverse(scene.bvh_sphere, ray, sphere_intersector, temp_rayhit, scene.trace_options);
    if (hit_sphere) {
      rayhit = temp_rayhit;
      hit += hit_sphere;
    }
  }

  return hit;
}

// Implementation of the add_ functions goes below ..
template<typename T>
unsigned int BlazertScene<T>::add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles) {

  // TODO: For now, only one mesh is supported ...
  if ((!has_mesh) && (!has_been_committed)) {
    triangles_ = TriangleMesh(vertices, triangles);
    triangles_sah_ = TriangleSAHPred(vertices, triangles);

    const unsigned int prim_id = primitives;
    primitives += triangles.size();
    has_mesh = true;
    has_been_committed = false;

    return prim_id;
  } else {
    return -1;
  }
}

template<typename T>
unsigned int BlazertScene<T>::add_spheres(const std::vector<Vec3r<T>> &centers, const std::vector<T> &radiuss) {

  if ((!has_sphere) && (!has_been_committed)) {
    spheres_ = Sphere(centers, radiuss);
    spheres_sah_ = SphereSAHPred(centers, radiuss);

    const unsigned int prim_id = primitives;
    primitives += 1;
    has_sphere = true;
    has_been_committed = false;
    return prim_id;
  } else {
    return -1;
  }
}

}// namespace blazert

#endif//BLAZERT_BLAZERT_SCENE_H
