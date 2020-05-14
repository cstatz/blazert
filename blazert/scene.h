#pragma once
#ifndef BLAZERT_BLAZERT_SCENE_H
#define BLAZERT_BLAZERT_SCENE_H

#include <blazert/bvh/accel.h>
#include <blazert/bvh/options.h>
#include <blazert/datatypes.h>

#include <blazert/primitives/plane.h>
#include <blazert/primitives/sphere.h>
#include <blazert/primitives/trimesh.h>

#include <blazert/ray.h>

namespace blazert {

template<typename T>
class BlazertScene {

public:
  BVHBuildOptions<T> build_options;
  BVHTraceOptions<T> trace_options;

  mutable bool has_been_committed = false;
  /***
   * geometries counts the amount of different geometry types
   * -> each geometry has its own BVH
   * -> for each geometry, we have various primitives; the hit prim_id will be saved in the RayHit structure
   ***/
  mutable unsigned int geometries = 0;

  TriangleMesh<T> triangles;
  TriangleSAHPred<T> triangles_sah;
  BVH<T> triangles_bvh;
  mutable bool has_triangles = false;

  Sphere<T> spheres;
  SphereSAHPred<T> spheres_sah;
  BVH<T> spheres_bvh;
  mutable bool has_spheres = false;

  Plane<T> planes;
  PlaneSAHPred<T> planes_sah;
  BVH<T> planes_bvh;
  mutable bool has_planes = false;
public:
  BlazertScene() = default;

  /**
   * @brief Adds a triangular mesh to the scene
   * @param vertices Vertices need to be allocated on the heap!
   * @param triangles Triangles need to be allocated on the heap!
   * @return Returns the (geometry) id for the mesh.
   * The prim_id is set in the rayhit structure by the intersection functions.
   */
  unsigned int add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles);

  /***
   * Adds centers.size() spheres to the scene -> results in centers.size() primitive ideas
   * @brief Adds spheres at centers with radii
   * @param centers specifies centers of the spheres (needs to be allocated on heap)
   * @param radii specifies radii of the spheres (needs to be allocated on heap)
   * @return geometry id of the spheres
   */
  unsigned int add_spheres(const Vec3rList<T> &centers, const std::vector<T> &radii);

  /***
   * @brief Adds planes at centers with dimensions dxs and dys rotated around rotations
   * @param centers center of planes
   * @param dxs dimensions in x direction
   * @param dys dimensions in y direction
   * @param rotations local rotation matrices
   * @return
   */
  unsigned int add_planes(const Vec3rList<T> &centers, const std::vector<T> &dxs, const std::vector<T> &dys, const Mat3rList<T> &rotations);

  bool commit() {

    if (has_triangles) {
      triangles_bvh.build(triangles, triangles_sah, build_options);
    }

    if (has_spheres) {
      spheres_bvh.build(spheres, spheres_sah, build_options);
    }

    if (has_planes) {
      planes_bvh.build(planes, planes_sah, build_options);
    }

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
  if (scene.has_triangles) {
    TriangleIntersector<T> triangle_intersector{*(scene.triangles.vertices), *(scene.triangles.faces)};
    const bool hit_mesh = traverse(scene.triangles_bvh, ray, triangle_intersector, temp_rayhit, scene.trace_options);
    if (hit_mesh) {
      rayhit = temp_rayhit;
      hit += hit_mesh;
    }
  }

  if (scene.has_spheres) {
    SphereIntersector<T> sphere_intersector{*(scene.spheres.centers), *(scene.spheres.radii)};
    const bool hit_sphere = traverse(scene.spheres_bvh, ray, sphere_intersector, temp_rayhit, scene.trace_options);
    if (hit_sphere) {
      if (temp_rayhit.hit_distance < rayhit.hit_distance) {
        rayhit = temp_rayhit;
        hit += hit_sphere;
      }
    }
  }

  if (scene.has_planes) {
    PlaneIntersector<T> plane_intersector{*(scene.planes.centers), *(scene.planes.dxs), *(scene.planes.dys), *(scene.planes.rotations)};
    const bool hit_plane = traverse(scene.planes_bvh, ray, plane_intersector, temp_rayhit, scene.trace_options);
    if (hit_plane) {
      if (temp_rayhit.hit_distance < rayhit.hit_distance) {
        rayhit = temp_rayhit;
        hit += hit_plane;
      }
    }
  }

  return hit;
}

// Implementation of the add_ functions goes below ..
template<typename T>
unsigned int BlazertScene<T>::add_mesh(const Vec3rList<T> &vertices, const Vec3iList &faces) {

  if ((!has_triangles) && (!has_been_committed)) {
    triangles = TriangleMesh(vertices, faces);
    triangles_sah = TriangleSAHPred(vertices, faces);
    has_triangles = true;

    return geometries++;
  } else {
    return -1;
  }
}

template<typename T>
unsigned int BlazertScene<T>::add_spheres(const Vec3rList<T> &centers, const std::vector<T> &radii) {

  if ((!has_spheres) && (!has_been_committed)) {
    spheres = Sphere(centers, radii);
    spheres_sah = SphereSAHPred(centers, radii);
    has_spheres = true;

    return geometries++;
  } else {
    return -1;
  }
}

template<typename T>
unsigned int BlazertScene<T>::add_planes(const Vec3rList<T> &centers, const std::vector<T> &dxs, const std::vector<T> &dys, const Mat3rList<T> &rotations) {

  if ((!has_planes) && (!has_been_committed)) {
    planes = Plane(centers, dxs, dys, rotations);
    planes_sah = PlaneSAHPred(centers, dxs, dys, rotations);
    has_planes = true;

    return geometries++;
  } else {
    return -1;
  }
}

}// namespace blazert

#endif//BLAZERT_BLAZERT_SCENE_H
