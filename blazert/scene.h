#pragma once
#ifndef BLAZERT_BLAZERT_SCENE_H
#define BLAZERT_BLAZERT_SCENE_H


#include <blazert/primitives/trimesh.h>
#include <blazert/bvh/accel.h>
#include <blazert/bvh/options.h>
#include <blazert/datatypes.h>
#include <blazert/ray.h>

namespace blazert {

template<typename T>
class Scene {
public:
  BVHBuildOptions<T> build_options;

private:
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

  BVHAccel<T> mesh_accel;
  //BVHAccel<T> sphere_accel;
  //BVHAccel<T> plane_accel;
  //BVHAccel<T> box_accel;
  //BVHAccel<T> cylinder_accel;

public:
  Scene() =default;

  // TODO: default arguments ...
  inline unsigned int add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles);
  //inline unsigned int add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles, Vec3rList &vertex_normals);
  //inline unsigned int add_sphere(const Vec3r<T> &center, const Vec3r<T> &radii, const Matrix3r<T> &rotation);
  //inline unsigned int add_plane(const Vec3r<T> &origin, const Vec3r<T> &bmin, const Vec3r<T> &bmax, const Matrix3r<T> &rotation);
  //inline unsigned int add_box(const Vec3r<T> &center, const Vec3r<T> &size, const Matrix3r<T> &rotation);

  //template<class X, ...> add_custom_primitive( ... );

  inline bool commit() {

    // Build all the BVH ...
    if (has_mesh) {
      mesh_accel.Build(triangles_, triangles_sah_, build_options);
    }

    //sphere_accel.Build()
    //place_accel.Build()
    //box_accel.Build()
    has_been_committed = true;
    return has_been_committed;
  };

  inline bool intersect1(const Ray<T> &ray, RayHit<T> rayhit) const {

    // This may not be optimal, but the interface is simple and everything can (and will) be in-lined.
    RayHit<T> temp_rayhit;
    bool hit = false;

    // Do the traversal for all primitives ...
    if (has_mesh) {
      TriangleIntersector<double> triangle_intersector(*triangles_.vertices_, *triangles_.faces_);
      bool meshhit = mesh_accel.Traverse(ray, triangle_intersector, temp_rayhit);
      if (meshhit) {
        rayhit = temp_rayhit;
        hit += meshhit;
      }
    }

    return hit;
  };
};

// Implementation of the add_ functions goes below ..
template<typename T>
inline unsigned int Scene<T>::add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles) {

  // TODO: For now, only one mesh is supported ...
  if ((!has_mesh) && (!has_been_committed)) {
    triangles_ = TriangleMesh(vertices, triangles);
    triangles_sah_ = TriangleSAHPred(vertices, triangles);

    const unsigned int prim_id = primitives;
    primitives += triangles.size();
    has_mesh = true;

    return prim_id;
  }
  else {
    return -1;
  }
};

}// namespace blazert

#endif//BLAZERT_BLAZERT_SCENE_H
