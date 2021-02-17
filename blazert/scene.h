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

/**
 * @brief BlazertScene provides the high-level interface for ray tracing with blazert
 *
 * @details
 *  BlazertScene is responsible for the entire ray tracing process. It provides measures to add different geometry types.
 *  Furthermore, it encapsulates the BVH for each geometry type. Parametrization is available for the floating point
 *  precision type. This is useful for scientific applications.
 *
 * @note This API is considered to be stable.
 *
 * @tparam T floating point type
 * @tparam BVH_T BVH type which is used to represent the BVH
 * @tparam Builder Type of the builder to use for building the BVH
 */
template<typename T, template<typename, template<typename> typename> typename BVH_T, typename Builder>
class BlazertScene {

public:
  BVHBuildOptions<T> build_options;

  bool has_been_committed = false;
  /*
   * geometries counts the amount of different geometry types
   * -> each geometry has its own BVH
   * -> for each geometry, we have various primitives; the hit prim_id will be saved in the RayHit structure
   */
  unsigned int geometries = 0;

  std::unique_ptr<TriangleMesh<T>> triangle_collection;
  std::unique_ptr<BVH<T, TriangleMesh>> triangles_bvh;
  unsigned int triangles_geom_id = static_cast<unsigned int>(-1);
  bool has_triangles = false;

  std::unique_ptr<SphereCollection<T>> sphere_collection;
  std::unique_ptr<BVH<T, SphereCollection>> spheres_bvh;
  unsigned int spheres_geom_id = static_cast<unsigned int>(-1);
  bool has_spheres = false;

  std::unique_ptr<PlaneCollection<T>> plane_collection;
  std::unique_ptr<BVH<T, PlaneCollection>> planes_bvh;
  unsigned int planes_geom_id = static_cast<unsigned int>(-1);
  bool has_planes = false;

  std::unique_ptr<CylinderCollection<T>> cylinder_collection;// these are needed for lifetime management...
  std::unique_ptr<BVH<T, CylinderCollection>> cylinders_bvh;
  unsigned int cylinders_geom_id = static_cast<unsigned int>(-1);
  bool has_cylinders = false;

public:
  BlazertScene() = default;

  unsigned int add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles);
  unsigned int add_spheres(const Vec3rList<T> &centers, const std::vector<T> &radii);
  unsigned int add_planes(const Vec3rList<T> &centers, const std::vector<T> &dxs, const std::vector<T> &dys,
                          const Mat3rList<T> &rotations);
  unsigned int add_cylinders(const Vec3rList<T> &centers, const std::vector<T> &semi_axes_a,
                             const std::vector<T> &semi_axes_b, const std::vector<T> &heights,
                             const Mat3rList<T> &rotations);
  /**
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
      Builder builder;
      builder.build(*triangles_bvh, build_options);
    }

    if (has_spheres) {
      Builder builder;
      builder.build(*spheres_bvh, build_options);
    }

    if (has_planes) {
      Builder builder;
      builder.build(*planes_bvh, build_options);
    }

    if (has_cylinders) {
      Builder builder;
      builder.build(*cylinders_bvh, build_options);
    }

    has_been_committed = true;
    return has_been_committed;
  };
};

template<typename T, template<typename, template<typename> typename> typename BVH_T, typename Builder>
std::ostream &operator<<(std::ostream &stream, const BlazertScene<T, BVH_T, Builder> &scene) {
  /// Conveniently output the scene as JSON
  stream << "{\n";

  stream << R"(  "Scene": )"
         << (&scene)
         << ",\n";
  stream << R"(  "Collections": [)"
         << "\n";

  if (scene.triangle_collection != nullptr) {
    stream << *scene.triangle_collection;
    if (scene.has_spheres || scene.has_cylinders || scene.has_planes) stream << ",\n";
  }
  if (scene.sphere_collection != nullptr) {
    stream << *scene.sphere_collection;
    if (scene.has_cylinders || scene.has_planes) stream << ",\n";
  }
  if (scene.cylinder_collection != nullptr) {
    stream << *scene.cylinder_collection;
    if (scene.has_planes) stream << ",\n";
  }
  if (scene.plane_collection != nullptr) stream << *scene.plane_collection;

  stream << "  ]\n";
  stream << "}\n";
  return stream;
}

/**
 * @brief Runs intersection tests for a given BlazertScene and Ray.
 *
 * @details
 *  intersect1 runs intersection test for 1 ray with the given scene. Thus, the BVH for each individual geometry is
 *  traversed until a hit is found.
 *
 * @tparam T floating point type, which is usually float or double, but in the future, quadruple precision might be useful
 * @param scene BlazertScene against which the ray is tested.
 * @param ray The Ray is used for the intersection testing.
 * @param rayhit RayHit structure to save the intersection data in.
 * @return True if a hit is found, otherwise false
 */
template<typename T, template<typename, template<typename> typename> typename BVH_T, typename Builder>
inline bool intersect1(const BlazertScene<T, BVH_T, Builder> &scene, const Ray<T> &ray, RayHit<T> &rayhit) {

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
template<typename T, template<typename, template<typename> typename> typename BVH_T, typename Builder>
unsigned int BlazertScene<T, BVH_T, Builder>::add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles) {

  if ((!has_triangles) && (!has_been_committed)) {
    triangle_collection = std::make_unique<TriangleMesh<T>>(vertices, triangles);
    triangles_bvh = std::make_unique<BVH_T<T, TriangleMesh>>(*triangle_collection);

    has_triangles = true;
    triangles_geom_id = geometries++;
    return triangles_geom_id;
  } else {
    return static_cast<unsigned int>(-1);
  }
}

/**
 * @brief Adds spheres at centers with radii.
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
 * @note centers and radii need to be allocated on the heap.
 * @note centers and spheres should be of the same length.
 */
template<typename T, template<typename, template<typename> typename> typename BVH_T, typename Builder>
unsigned int BlazertScene<T, BVH_T, Builder>::add_spheres(const Vec3rList<T> &centers, const std::vector<T> &radii) {

  if ((!has_spheres) && (!has_been_committed)) {
    sphere_collection = std::make_unique<SphereCollection<T>>(centers, radii);
    spheres_bvh = std::make_unique<BVH_T<T, SphereCollection>>(*sphere_collection);

    has_spheres = true;
    spheres_geom_id = geometries++;
    return spheres_geom_id;
  } else {
    return static_cast<unsigned int>(-1);
  }
}

/**
 * @brief Adds planes at centers with dimensions dxs and dys rotated around rotations.
 *
 * @details
 *  The planes are described by centers, dimensions in x and y direction as well as rotation matrices.
 *  For \f$N\f$ planes, each of these vectors needs to have \f$N\f$ entries describing the corresponding planes'
 *  centers, dimensions in x and y direction and rotation matrices.
 *
 *  The prim_id is set in the rayhit structure by the intersection functions.
 *
 * @param centers center of planes
 * @param dxs dimensions in x direction
 * @param dys dimensions in y direction
 * @param rotations local rotation matrices
 *
 * @return geometry id of the planes
 *
 * @note centers, dxy, dys, and rotations need to be allocated on the heap.
 * @note centers, dxy, dys, and rotations should be of the same length.
 */
template<typename T, template<typename, template<typename> typename> typename BVH_T, typename Builder>
unsigned int BlazertScene<T, BVH_T, Builder>::add_planes(const Vec3rList<T> &centers, const std::vector<T> &dxs,
                                         const std::vector<T> &dys, const Mat3rList<T> &rotations) {

  if ((!has_planes) && (!has_been_committed)) {
    plane_collection = std::make_unique<PlaneCollection<T>>(centers, dxs, dys, rotations);
    planes_bvh = std::make_unique<BVH_T<T, PlaneCollection>>(*plane_collection);

    has_planes = true;
    planes_geom_id = geometries++;
    return planes_geom_id;
  } else {
    return static_cast<unsigned int>(-1);
  }
}

/**
 * @brief Adds cylinders at centers, described by two semi_axes and heights.
 *
 * @details
 *  The cylinders are described by their centers, two semi-axes describing the ellipoidal shape of the top and bottom
 *  their height and rotations. For \f$N\f$ planes, each of these vectors needs to have \f$N\f$ entries describing the
 *  corresponding cylinderes' centers, dimensions in x and y direction and rotation matrices.
 *
 *  The prim_id is set in the rayhit structure by the intersection functions.
 *
 * @param centers centers of the cylinders
 * @param semi_axes_a semi-axes in x-direction
 * @param semi_axes_b semi-axes in y-direction
 * @param heights heights of the cylinders
 * @param rotations rotation matrices
 *
 * @return geometry id of the cylinders
 *
 * @note centers, dxy, dys, and rotations need to be allocated on the heap.
 * @note centers, dxy, dys, and rotations should be of the same length.
 *
 */
template<typename T, template<typename, template<typename> typename> typename BVH_T, typename Builder>
unsigned int BlazertScene<T, BVH_T, Builder>::add_cylinders(const Vec3rList<T> &centers, const std::vector<T> &semi_axes_a,
                                            const std::vector<T> &semi_axes_b, const std::vector<T> &heights,
                                            const Mat3rList<T> &rotations) {
  if ((!has_cylinders) && (!has_been_committed)) {
    cylinder_collection =
        std::make_unique<CylinderCollection<T>>(centers, semi_axes_a, semi_axes_b, heights, rotations);
    cylinders_bvh = std::make_unique<BVH_T<T, CylinderCollection>>(*cylinder_collection);
    has_cylinders = true;

    cylinders_geom_id = geometries++;
    return cylinders_geom_id;
  } else {
    return static_cast<unsigned int>(-1);
  }
}

}// namespace blazert

#endif//BLAZERT_BLAZERT_SCENE_H
