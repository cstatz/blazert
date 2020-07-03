/**
 * @file scene.h
 * @brief High level scene interface for ray tracing
 * @authors Christoph Statz, Orell Garten
 */

#pragma once
#ifndef BLAZERT_BLAZERT_SCENE_H
#define BLAZERT_BLAZERT_SCENE_H

#include <blazert/bvh/accel.h>
#include <blazert/bvh/builder.h>
#include <blazert/bvh/options.h>
#include <blazert/datatypes.h>

#include <blazert/primitives/cylinder.h>
#include <blazert/primitives/plane.h>
#include <blazert/primitives/sphere.h>
#include <blazert/primitives/trimesh.h>

#include <blazert/bvh/builder.h>
#include <blazert/ray.h>

namespace blazert {

template<typename T>
class BlazertScene {

public:
  BVHBuildOptions<T> build_options;

  bool has_been_committed = false;
  /**
   * geometries counts the amount of different geometry types
   * -> each geometry has its own BVH
   * -> for each geometry, we have various primitives; the hit prim_id will be saved in the RayHit structure
   **/
  unsigned int geometries = 0;

  std::unique_ptr<TriangleMesh<T>> triangle_collection;
  std::unique_ptr<BVH<T, TriangleMesh>> triangles_bvh;
  size_t triangles_geom_id = -1;
  bool has_triangles = false;

  std::unique_ptr<SphereCollection<T>> sphere_collection;
  std::unique_ptr<BVH<T, SphereCollection>> spheres_bvh;
  size_t spheres_geom_id = -1;
  bool has_spheres = false;

  std::unique_ptr<PlaneCollection<T>> plane_collection;
  std::unique_ptr<BVH<T, PlaneCollection>> planes_bvh;
  size_t planes_geom_id = -1;
  bool has_planes = false;

  std::unique_ptr<CylinderCollection<T>> cylinder_collection;// these are needed for lifetime management...
  std::unique_ptr<BVH<T, CylinderCollection>> cylinders_bvh;
  size_t cylinders_geom_id = -1;
  bool has_cylinders = false;

public:
  BlazertScene() = default;

  /**
   * @brief Adds a triangular mesh to the scene
   * @details
   *  A triangular mesh can be used to describe any geometry. The mesh is described by vertices and a list of triangles
   *  which describe which vertices form a triangle.
   *  The prim_id is set in the rayhit structure by the intersection functions.
   *
   * @param vertices Vertices need to be allocated on the heap!
   * @param triangles Triangles need to be allocated on the heap!
   * @return geometry id for the mesh.
   *
   * @note vertices and triangles need to be allocated on the heap.
   *
   */
  unsigned int add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles);

  /**
   * @brief Adds spheres at centers with radii
   *
   * @details
   *  The spheres are described by centers and radii. For \f$N\f$ spheres, each of these vectors
   *  needs to have \f$N\f$ entries describing the corresponding sphere's center and radius.
   *
   *  The prim_id is set in the rayhit structure by the intersection functions.
   *
   * @param centers specifies centers of the spheres (needs to be allocated on heap)
   * @param radii specifies radii of the spheres (needs to be allocated on heap)
   * @return geometry id of the spheres
   *

   *
   * @note centers and radii need to be allocated on the heap.
   * @note centers and spheres should be of the same length.
   */
  unsigned int add_spheres(const Vec3rList<T> &centers, const std::vector<T> &radii);

  /**
   * @brief Adds planes at centers with dimensions dxs and dys rotated around rotations
   * @param centers center of planes
   * @param dxs dimensions in x direction
   * @param dys dimensions in y direction
   * @param rotations local rotation matrices
   * @return geometry id of the planes
   */
  unsigned int add_planes(const Vec3rList<T> &centers, const std::vector<T> &dxs, const std::vector<T> &dys,
                          const Mat3rList<T> &rotations);

  /**
   * @brief Adds cylinders with the bottom ellipsoid centered at centers, described by two semi_axes and heights.
   * @param centers
   * @param semi_axes_a
   * @param semi_axes_b
   * @param heights
   * @param rotations
   * @return geometry id of the cylinders
   */
  unsigned int add_cylinders(const Vec3rList<T> &centers, const std::vector<T> &semi_axes_a,
                             const std::vector<T> &semi_axes_b, const std::vector<T> &heights,
                             const Mat3rList<T> &rotations);
  /**
   * This function commits the scene
   *
   * @brief Commits the scene and builds BVH for each geometry type
   *
   * @details
   * It is necessary to run this function before running the intersect functions. The bounding volume hierarchy
   * is built for each geometry type present in the scene.
   *
   * @return returns true if scene has been committed
   */
  bool commit() {

    if (has_triangles) {
      SAHBinnedBuilder builder;
      builder.build(*triangles_bvh, build_options);
    }

    if (has_spheres) {
      SAHBinnedBuilder builder;
      builder.build(*spheres_bvh, build_options);
    }

    if (has_planes) {
      SAHBinnedBuilder builder;
      builder.build(*planes_bvh, build_options);
    }

    if (has_cylinders) {
      SAHBinnedBuilder builder;
      builder.build(*cylinders_bvh, build_options);
    }

    has_been_committed = true;
    return has_been_committed;
  };
};

/**
 *
 * @tparam T floating point type, which is usually float or double, but in the future, quadruple precision might be useful
 * @param scene
 * @param ray
 * @param rayhit
 * @return
 */
template<typename T>
inline bool intersect1(const BlazertScene<T> &scene, const Ray<T> &ray, RayHit<T> &rayhit) {

  // This may not be optimal, but the interface is simple and everything can (and will) be in-lined.
  RayHit<T> temp_rayhit;
  bool hit = false;

  // Do the traversal for all primitives ...
  if (scene.has_triangles) {
    const bool hit_mesh = traverse(*scene.triangles_bvh, ray, temp_rayhit);
    if (hit_mesh) {
      rayhit = temp_rayhit;
      rayhit.geom_id = scene.triangles_geom_id;
      hit += hit_mesh;
    }
  }

  if (scene.has_spheres) {
    const bool hit_sphere = traverse(*scene.spheres_bvh, ray, temp_rayhit);
    if (hit_sphere) {
      if (temp_rayhit.hit_distance < rayhit.hit_distance) {
        rayhit = temp_rayhit;
        rayhit.geom_id = scene.spheres_geom_id;
        hit += hit_sphere;
      }
    }
  }

  if (scene.has_planes) {
    const bool hit_plane = traverse(*scene.planes_bvh, ray, temp_rayhit);
    if (hit_plane) {
      if (temp_rayhit.hit_distance < rayhit.hit_distance) {
        rayhit = temp_rayhit;
        rayhit.geom_id = scene.planes_geom_id;
        hit += hit_plane;
      }
    }
  }

  if (scene.has_cylinders) {
    const bool hit_cylinder = traverse(*scene.cylinders_bvh, ray, temp_rayhit);
    if (hit_cylinder) {
      if (temp_rayhit.hit_distance < rayhit.hit_distance) {
        rayhit = temp_rayhit;
        rayhit.geom_id = scene.cylinders_geom_id;
        hit += hit_cylinder;
      }
    }
  }
  return hit;
}

// Implementation of the add_ functions goes below ..
template<typename T>
unsigned int BlazertScene<T>::add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles) {

  if ((!has_triangles) && (!has_been_committed)) {
    triangle_collection = std::make_unique<TriangleMesh<T>>(vertices, triangles);
    triangles_bvh = std::make_unique<BVH<T, TriangleMesh>>(*triangle_collection);

    has_triangles = true;
    triangles_geom_id = geometries++;
    return triangles_geom_id;
  } else {
    return static_cast<unsigned int>(-1);
  }
}

template<typename T>
unsigned int BlazertScene<T>::add_spheres(const Vec3rList<T> &centers, const std::vector<T> &radii) {

  if ((!has_spheres) && (!has_been_committed)) {
    sphere_collection = std::make_unique<SphereCollection<T>>(centers, radii);
    spheres_bvh = std::make_unique<BVH<T, SphereCollection>>(*sphere_collection);

    has_spheres = true;
    spheres_geom_id = geometries++;
    return spheres_geom_id;
  } else {
    return static_cast<unsigned int>(-1);
  }
}

template<typename T>
unsigned int BlazertScene<T>::add_planes(const Vec3rList<T> &centers, const std::vector<T> &dxs,
                                         const std::vector<T> &dys, const Mat3rList<T> &rotations) {

  if ((!has_planes) && (!has_been_committed)) {
    plane_collection = std::make_unique<PlaneCollection<T>>(centers, dxs, dys, rotations);
    planes_bvh = std::make_unique<BVH<T, PlaneCollection>>(*plane_collection);

    has_planes = true;
    planes_geom_id = geometries++;
    return planes_geom_id;
  } else {
    return -1;
  }
}
template<typename T>
unsigned int BlazertScene<T>::add_cylinders(const Vec3rList<T> &centers, const std::vector<T> &semi_axes_a,
                                            const std::vector<T> &semi_axes_b, const std::vector<T> &heights,
                                            const Mat3rList<T> &rotations) {
  if ((!has_cylinders) && (!has_been_committed)) {
    cylinder_collection =
        std::make_unique<CylinderCollection<T>>(centers, semi_axes_a, semi_axes_b, heights, rotations);
    cylinders_bvh = std::make_unique<BVH<T, CylinderCollection>>(*cylinder_collection);
    has_cylinders = true;

    cylinders_geom_id = geometries++;
    return cylinders_geom_id;
  } else {
    return -1;
  }
}

}// namespace blazert

#endif//BLAZERT_BLAZERT_SCENE_H
