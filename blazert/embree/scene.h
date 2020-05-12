#pragma once
#ifndef BLAZERT_EMBREE_SCENE_H
#define BLAZERT_EMBREE_SCENE_H

#include <blazert/datatypes.h>
#include <blazert/ray.h>
#include <blazert/scene.h>
#include <embree3/rtcore.h>

namespace blazert {

template<typename T>
class EmbreeScene {
private:
  RTCDevice device;

public:
  RTCScene scene;
  BlazertScene<T> b_scene;
  bool has_been_committed;

  EmbreeScene() : device(rtcNewDevice("verbose=0,start_threads=1,threads=4,set_affinity=1")),
                  scene(rtcNewScene(device)), has_been_committed(false) {
    if constexpr(std::is_same<double, T>::value) {
      std::cout << "-> Attention: Using embree bvh and traversal for float tracing. <-" << std::endl;
      std::cout << "This will also impact double precision tracing by blazert." << std::endl;
      std::cout << "You're using doubles for geometry computation, consider disabling embree." << std::endl;
    }
    rtcSetSceneFlags(scene, RTC_SCENE_FLAG_NONE);
  };

  unsigned int add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles);

  //template<class X, ...> add_custom_primitive( ... );

  bool commit() {
    if (!has_been_committed) {

      if constexpr (std::is_same<float, T>::value) {
        has_been_committed = true;
        rtcCommitScene(scene);
      } else {
        has_been_committed = b_scene.commit();
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

    const RTCRay r{ray.origin[0], ray.origin[1], ray.origin[2], ray.min_hit_distance, ray.direction[0], ray.direction[1], ray.direction[2], 0, ray.max_hit_distance, 0, 0, 0};
    RTCHit h{};
    RTCRayHit rh{r, h};
    rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(scene.scene, &context, &rh);

    if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
      hit = true;
      rayhit.prim_id = rh.hit.primID;
      rayhit.uv = {rh.hit.u, rh.hit.v};
      rayhit.hit_distance = rh.ray.tfar;
    }
  } else {
    hit = intersect1(scene.b_scene, ray, rayhit);
  }

  return hit;
};

template<typename T>
unsigned int EmbreeScene<T>::add_mesh(const Vec3rList<T> &vertices, const Vec3iList &triangles) {

  unsigned int id = -1;

  if constexpr (std::is_same<float, T>::value) {
    constexpr const int bytestride_int = sizeof(Vec3ui) / 4 * sizeof(Vec3ui::ElementType);
    constexpr const int bytestride_float = sizeof(Vec3r<float>) / 4 * sizeof(Vec3r<float>::ElementType);

    auto geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    rtcSetSharedGeometryBuffer(geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, (void *) (triangles.data()), 0, bytestride_int, triangles.size());
    rtcSetSharedGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, (void *) (vertices.data()), 0, bytestride_float, vertices.size());
    rtcCommitGeometry(geometry);
    auto geom_id = rtcAttachGeometry(scene, geometry);
    id = geom_id;

  } else {
    id = b_scene.add_mesh(vertices, triangles);
  }
  return id;
}

}// namespace blazert

#endif//BLAZERT_EMBREE_SCENE_H
