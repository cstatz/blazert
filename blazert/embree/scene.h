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

  //template<class X, ...> add_custom_primitive( ... );

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

// TODO: Performance critical code should not be a member function (hidden pointer *this), since the compiler will sometimes not know how to optimize.
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
};

template<typename T>
unsigned int EmbreeScene<T>::add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles) {

  unsigned int id = -1;

  if constexpr (std::is_same<float, T>::value) {
    constexpr const int bytestride_int = sizeof(Vec3ui) / 8 * sizeof(Vec3ui::ElementType);
    constexpr const int bytestride_float = sizeof(Vec3r<float>) / 8 * sizeof(Vec3r<float>::ElementType);

    triangle_mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    rtcSetSharedGeometryBuffer(triangle_mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, (void *) (triangles.data()),
                               0, bytestride_int, triangles.size());
    rtcSetSharedGeometryBuffer(triangle_mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, (void *) (vertices.data()),
                               0, bytestride_float, vertices.size());
    rtcCommitGeometry(triangle_mesh);
    auto geom_id = rtcAttachGeometry(rtcscene, triangle_mesh);
    id = geom_id;
  } else {
    id = blazertscene.add_mesh(vertices, triangles);
  }
  return id;
}

template<typename T>
unsigned int EmbreeScene<T>::add_spheres(const Vec3rList<T> &centers, const std::vector<T> &radii) {

  unsigned int id = -1;

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

template<typename T>
unsigned int EmbreeScene<T>::add_planes(const Vec3rList<T> &centers, const std::vector<T> &dxs,
                                        const std::vector<T> &dys, const Mat3rList<T> &rotations) {

  unsigned int id = -1;

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

template<typename T>
unsigned int EmbreeScene<T>::add_cylinders(const Vec3rList<T> &centers, const std::vector<T> &semi_axes_a,
                                           const std::vector<T> &semi_axes_b, const std::vector<T> &heights,
                                           const Mat3rList<T> &rotations) {
  unsigned int id = -1;

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
