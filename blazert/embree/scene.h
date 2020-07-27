#pragma once
#ifndef BLAZERT_EMBREE_SCENE_H
#define BLAZERT_EMBREE_SCENE_H

#include <blazert/datatypes.h>
#include <blazert/ray.h>
#include <blazert/scene.h>

#include <blazert/embree/primitives/EmbreeCylinder.h>
#include <blazert/embree/primitives/EmbreePlane.h>
#include <blazert/embree/primitives/EmbreeSphere.h>

#include <embree3/rtcore.h>
//#include <map>

namespace blazert {

/**
 * @brief EmbreeScene provides the high-level interface for ray tracing with embree backend.
 *
 * @details
 *  EmbreeScene is responsible for the entire ray tracing process. It provides measures to add different geometry types.
 *  Furthermore, it encapsulates the BVH for each geometry type.
 *
 *  @warning
 *  - If T is a single precision type, the embree backend will be used to do the ray tracing.
 *  - If T is a double precision type, the blazert backend will be used (embree **won't** be used in this case)
 *
 * @note This API is considered to be stable.
 *
 * @tparam T floating point type
 */
template<typename T>
class EmbreeScene {
private:
  RTCDevice device;

public:
  RTCScene rtcscene;

  RTCGeometry triangle_mesh;
  std::unique_ptr<EmbreeSphere> sphere;
  std::unique_ptr<EmbreeCylinder> cylinder;
  std::unique_ptr<EmbreePlane> plane;

  BlazertScene<T> blazertscene;
  bool has_been_committed;

  EmbreeScene()
      : device(rtcNewDevice("verbose=0,start_threads=1,threads=4,set_affinity=1")), rtcscene(rtcNewScene(device)),
        has_been_committed(false) {
    if constexpr (std::is_same<double, T>::value) {
      std::cout << "-> Attention: Using embree bvh and traversal for float tracing. <-" << std::endl;
      std::cout << "This will also impact double precision tracing by blazert." << std::endl;
      std::cout << "You're using doubles for geometry computation, consider disabling embree." << std::endl;
    }
    rtcSetSceneFlags(rtcscene, RTC_SCENE_FLAG_NONE);
  };

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
    if (!has_been_committed) {

      if constexpr (std::is_same<float, T>::value) {
        has_been_committed = true;
        rtcCommitScene(rtcscene);
      } else {
        has_been_committed = blazertscene.commit();
      }
    }
    return has_been_committed;
  };
};

/**
 * @brief  sdfsdfdsfs
 *
 * @details bdsdf
 *
 * @tparam T floating point type, which is usually float or double, but in the future, quadruple precision might be useful
 * @param scene
 * @param ray
 * @param rayhit
 * @return True if a hit is found, otherwise false
 */
template<typename T>
inline bool intersect1(const EmbreeScene<T> &scene, const Ray<T> &ray, RayHit<T> &rayhit) {

  // This surely generates a little overhead for embree.
  bool hit = false;

  if constexpr (std::is_same<float, T>::value) {
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    const RTCRay r{ray.origin[0],
                   ray.origin[1],
                   ray.origin[2],
                   ray.min_hit_distance,
                   ray.direction[0],
                   ray.direction[1],
                   ray.direction[2],
                   0,
                   ray.max_hit_distance,
                   0,
                   0,
                   0};
    RTCHit h{};
    RTCRayHit rh{r, h};
    rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(scene.rtcscene, &context, &rh);

    if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
      hit = true;
      rayhit.prim_id = rh.hit.primID;
      rayhit.uv = {rh.hit.u, rh.hit.v};
      rayhit.hit_distance = rh.ray.tfar;
      rayhit.normal = Vec3r<T>{rh.hit.Ng_x, rh.hit.Ng_y, rh.hit.Ng_z};
    }
  } else {
    hit = intersect1(scene.blazertscene, ray, rayhit);
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
template<typename T>
unsigned int EmbreeScene<T>::add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles) {

  unsigned int id = static_cast<unsigned int>(-1);

  if constexpr (std::is_same<float, T>::value) {
    constexpr const int bytestride_int = sizeof(Vec3ui) / 8 * sizeof(Vec3ui::ElementType);
    constexpr const int bytestride_float = sizeof(Vec3r<float>) / 8 * sizeof(Vec3r<float>::ElementType);

    triangle_mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    rtcSetSharedGeometryBuffer(triangle_mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, reinterpret_cast<const void *>(triangles.data()),
                               0, bytestride_int, triangles.size());
    rtcSetSharedGeometryBuffer(triangle_mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, reinterpret_cast<const void *>(vertices.data()),
                               0, bytestride_float, vertices.size());
    rtcCommitGeometry(triangle_mesh);
    auto geom_id = rtcAttachGeometry(rtcscene, triangle_mesh);
    id = geom_id;
  } else {
    id = blazertscene.add_mesh(vertices, triangles);
  }
  return id;
}

/**
 * @brief Adds spheres at centers with radii
 *
 * @details
 *  The spheres are described by centers and radii. For \f$N\f$ spheres, each of these vectors
 *  needs to have \f$N\f$ entries describing the corresponding sphere's center and radius.
 *
 *  The prim_id is set in the rayhit structure by the intersection functions.
 *
 * @warning currently, only one sphere can be added
 *
 * @param centers specifies centers of the spheres (needs to be allocated on heap)
 * @param radii specifies radii of the spheres (needs to be allocated on heap)
 * @return geometry id of the spheres
 *
 * @note centers and radii need to be allocated on the heap.
 * @note centers and spheres should be of the same length.
 */
template<typename T>
unsigned int EmbreeScene<T>::add_spheres(const Vec3rList<T> &centers, const std::vector<T> &radii) {

  unsigned int id = static_cast<unsigned int>(-1);

  if constexpr (std::is_same<float, T>::value) {
    // TODO: We are looking for something more like this:
    sphere = std::make_unique<EmbreeSphere>(device, rtcscene, centers[0], radii[0]);
    id = sphere->geomID;
    //    for(size_t prim_id = 0; prim_id < centers.size(); ++prim_id) {
    //      const Vec3r<T> &c = centers[prim_id];
    //      const T r = radii[prim_id];
    //      auto sphere = std::make_unique<EmbreeSphere>(device, rtcscene, c, r);
    //      // TODO: This is probably not really...good
    //      id = sphere->geomID;
    //    }
  } else {
    id = blazertscene.add_spheres(centers, radii);
  }
  return id;
}

/**
 * @brief Adds planes at centers with dimensions dxs and dys rotated around rotations
 *
 * @details
 *  The planes are described by centers, dimensions in x and y direction as well as rotation matrices.
 *  For \f$N\f$ planes, each of these vectors needs to have \f$N\f$ entries describing the corresponding planes'
 *  centers, dimensions in x and y direction and rotation matrices.
 *
 *  The prim_id is set in the rayhit structure by the intersection functions.
 *
 * @warning currently, only one plane can be added
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
template<typename T>
unsigned int EmbreeScene<T>::add_planes(const Vec3rList<T> &centers, const std::vector<T> &dxs,
                                        const std::vector<T> &dys, const Mat3rList<T> &rotations) {

  unsigned int id = static_cast<unsigned int>(-1);

  if constexpr (std::is_same<float, T>::value) {
    plane = std::make_unique<EmbreePlane>(device, rtcscene, centers[0], dxs[0], dys[0], rotations[0]);
    id = plane->geomID;
    // TODO: We are looking for something more like this:
    //    for(size_t prim_id = 0; prim_id < centers.size(); ++prim_id) {
    //      const Vec3r<T> &c = centers[prim_id];
    //      const T dx = dxs[prim_id];
    //      const T dy = dys[prim_id];
    //      const Mat3r<T> &rot = rotations[prim_id];
    //      auto plane = std::make_unique<EmbreePlane>(device, rtcscene, c, dx, dy, rot);
    //      // TODO: This is probably not really...good
    //      id = plane->geomID;
    //    }
  } else {
    id = blazertscene.add_planes(centers, dxs, dys, rotations);
  }
  return id;
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
 *  @warning currently, only one cylinder can be added
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
 */
template<typename T>
unsigned int EmbreeScene<T>::add_cylinders(const Vec3rList<T> &centers, const std::vector<T> &semi_axes_a,
                                           const std::vector<T> &semi_axes_b, const std::vector<T> &heights,
                                           const Mat3rList<T> &rotations) {
  unsigned int id = static_cast<unsigned int>(-1);

  if constexpr (std::is_same<float, T>::value) {
    cylinder = std::make_unique<EmbreeCylinder>(device, rtcscene, centers[0], semi_axes_a[0], semi_axes_b[0],
                                                heights[0], rotations[0]);
    id = cylinder->geomID;
    // TODO: We are looking for something more like this:
    //    for(size_t prim_id = 0; prim_id < centers.size(); ++prim_id) {
    //      const Vec3r<T> &c = centers[prim_id];
    //      const T dx = dxs[prim_id];
    //      const T dy = dys[prim_id];
    //      const Mat3r<T> &rot = rotations[prim_id];
    //      auto plane = std::make_unique<EmbreePlane>(device, rtcscene, c, dx, dy, rot);
    //      // TODO: This is probably not really...good
    //      id = plane->geomID;
    //    }
  } else {
    id = blazertscene.add_cylinders(centers, semi_axes_a, semi_axes_b, heights, rotations);
  }
  return id;
}

}// namespace blazert

#endif//BLAZERT_EMBREE_SCENE_H
