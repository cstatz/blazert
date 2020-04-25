#pragma once
#ifndef BLAZERT_BLAZERT_SCENE_H
#define BLAZERT_BLAZERT_SCENE_H

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
  mutable bool has_been_commited = false;
  mutable bool has_mesh = false;
  mutable bool has_sphere = false;
  mutable bool has_plane = false;
  mutable bool has_box = false;
  mutable bool has_cylinder = false;
  mutable unsigned int primitives = 0;

  TriangleMesh<T> trianlges;
  TriangleMeshSAHPred<T> triangles_sah;

  BVHAccel<T> mesh_accel;
  //BVHAccel<T> sphere_accel;
  //BVHAccel<T> plane_accel;
  //BVHAccel<T> box_accel;
  //BVHAccel<T> cylinder_accel;

public:
  Scene(){};
  ~Scene(){};

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
      mesh_accesl.Build(triangles, triangles_sah, build_options)
    }

    //sphere_accel.Build()
    //place_accel.Build()
    //box_accel.Build()
    has_been_commited = true;
  };

  inline bool intersect1(const Ray<T> &ray, RayHit<T> rayhit) const {

    // This may not be optimal, but the interface is simple and everything can (and will) be in-lined.
    RayHit temp_rayhit;
    bool hit = false;

    // Do the traversal for all primitives ...
    if (has_mesh) {
      TriangleIntersector<double> triangle_intersector(mesh->vertices, mesh->triangles);
      RayHit<double> rayhit;
      bool mesh_hit = mesh_accel.Traverse(ray, triangle_intersector, temp_rayhit);
      if (mesh_hit) {
        ray_hit = temp_rayhit;
        hit += mesh_hit;
      }
    }

    return hit;
  };
};

// Implementation of the add_ functions goes below ..
template<typename T>
inline unsigned int Scene::add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles_) {

  // TODO: For now, only one mesh is supported ...
  if ((!has_mesh) && (!has_been_commited)) {
    triangles = TriangleMesh(vertices, triangles_);
    triangles_sah = TriangleMeshSAHPred(vertices, triangles_);

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
