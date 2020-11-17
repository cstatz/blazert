//
// Created by ogarten on 09/06/2020.
//

#ifndef BLAZERT_ASSERT_HELPER_H
#define BLAZERT_ASSERT_HELPER_H

#define DOCTEST_CONFIG_INCLUDE_TYPE_TRAITS

#include <blazert/bvh/accel.h>
#include <blazert/bvh/builder.h>
#include <blazert/datatypes.h>
#include <blazert/ray.h>
#include <third_party/doctest/doctest/doctest.h>

using namespace blazert;
using namespace doctest;

template<typename T, template<typename> typename Collection>
inline void assert_bounding_box(const Collection<T> &collection, const unsigned int prim_id, const Vec3r<T> &true_min,
                                const Vec3r<T> &true_max) {
  const auto [bmin, bmax] = collection.get_primitive_bounding_box(prim_id);

  CHECK(bmin[0] == Approx(true_min[0]));
  CHECK(bmin[1] == Approx(true_min[1]));
  CHECK(bmin[2] == Approx(true_min[2]));
  CHECK(bmax[0] == Approx(true_max[0]));
  CHECK(bmax[1] == Approx(true_max[1]));
  CHECK(bmax[2] == Approx(true_max[2]));
}

/**
    If multiple IDs (more objects) are used, all are checked and a global minimum and maximum are searched for
    @param num_prim_id : Number of prim_ids in object
*/
template<typename T, template<typename> typename Collection>
inline void assert_bounding_box_multi_prim_id(const Collection<T> &collection, const unsigned int prim_id,
                                              const Vec3r<T> &true_min, const Vec3r<T> &true_max,
                                              const unsigned int num_prim_id) {

  const auto [bmin_, bmax_] = collection.get_primitive_bounding_box(prim_id);

  Vec3r<T> bmin{bmin_[0], bmin_[1], bmin_[2]};
  Vec3r<T> bmax{bmax_[0], bmax_[1], bmax_[2]};

  for (unsigned int i = 0; i < num_prim_id; i++) {
    const auto [bmin_temp, bmax_temp] = collection.get_primitive_bounding_box(i);

    for (unsigned int k = 0; k < 3; k++) {
      bmin[k] = std::min(bmin[k], bmin_temp[k]);
      bmax[k] = std::max(bmax[k], bmax_temp[k]);
    }
  }

  CHECK(bmin[0] == Approx(true_min[0]));
  CHECK(bmin[1] == Approx(true_min[1]));
  CHECK(bmin[2] == Approx(true_min[2]));
  CHECK(bmax[0] == Approx(true_max[0]));
  CHECK(bmax[1] == Approx(true_max[1]));
  CHECK(bmax[2] == Approx(true_max[2]));
}

template<typename T, template<typename> typename Collection>
inline void assert_primitive_center(const Collection<T> &collection, const unsigned int prim_id,
                                    const Vec3r<T> &true_center) {
  const auto center = collection.get_primitive_center(prim_id);

  CHECK(center[0] == Approx(true_center[0]));
  CHECK(center[1] == Approx(true_center[1]));
  CHECK(center[2] == Approx(true_center[2]));
}

template<typename T, template<typename> typename Collection>
inline void assert_distance_to_surface(const Collection<T> &collection, const unsigned int prim_id,
                                       const Vec3r<T> &point, const T true_distance) {
  CHECK(collection.distance_to_surface(point, prim_id) == Approx(true_distance));
}

template<typename T, template<typename> typename Collection>
inline void assert_intersect_primitive_hit(const Collection<T> &collection, const Ray<T> &ray, const bool true_hit,
                                           const unsigned int prim_id, const T distance, const Vec3r<T> &normal) {
  typename Collection<T>::intersector intersector(collection);
  RayHit<T> rayhit;
  // Test intersections
  prepare_traversal(intersector, ray);
  const bool hit = intersect_primitive(intersector, primitive_from_collection(collection, 0), ray);
  if (hit)
    post_traversal(intersector, rayhit);

  CHECK(hit == true_hit);
  CHECK(rayhit.prim_id == prim_id);
  CHECK(rayhit.hit_distance == Approx(static_cast<T>(distance)));
  CHECK(rayhit.normal[0] == Approx(static_cast<T>(normal[0])));
  CHECK(rayhit.normal[1] == Approx(static_cast<T>(normal[1])));
  CHECK(rayhit.normal[2] == Approx(static_cast<T>(normal[2])));
}


/**
  * Used for trimesh, if more than one triangle is used
  */
template<typename T, template<typename> typename Collection>
inline void assert_traverse_bvh_hit_trimesh(const Collection<T> &collection, const Ray<T> &ray, const bool true_hit,
                                            const T distance, const Vec3r<T> &normal) {

  BVH bvh(collection);
  SAHBinnedBuilder builder;

  [[maybe_unused]] auto statistics = builder.build(bvh);

  RayHit<T> rayhit;
  const bool hit = traverse(bvh, ray, rayhit);
  CHECK(hit == true_hit);
  CHECK(rayhit.hit_distance == Approx(static_cast<T>(distance)));
  CHECK(rayhit.normal[0] == Approx(static_cast<T>(normal[0])));
  CHECK(rayhit.normal[1] == Approx(static_cast<T>(normal[1])));
  CHECK(rayhit.normal[2] == Approx(static_cast<T>(normal[2])));
}

/**
 * Temporary used for Cube mesh, for hit check and right distance to surface
 * TODO: Find out how to correctly check normals of cube mesh
 */
template<typename T, template<typename> typename Collection>
inline void assert_traverse_bvh_hit_trimesh_distance(const Collection<T> &collection, const Ray<T> &ray,
                                                 const bool true_hit, const T distance) {

  BVH bvh(collection);
  SAHBinnedBuilder builder;

  [[maybe_unused]] auto statistics = builder.build(bvh);

  RayHit<T> rayhit;
  const bool hit = traverse(bvh, ray, rayhit);
  CHECK(hit == true_hit);
  CHECK(rayhit.hit_distance == Approx(static_cast<T>(distance)));
}

template<typename T, template<typename> typename Collection>
inline void assert_traverse_bvh_hit(const Collection<T> &collection, const Ray<T> &ray, const bool true_hit,
                                    const unsigned int prim_id, const T distance, const Vec3r<T> &normal) {

  BVH bvh(collection);
  SAHBinnedBuilder builder;

  [[maybe_unused]] auto statistics = builder.build(bvh);

  RayHit<T> rayhit;
  const bool hit = traverse(bvh, ray, rayhit);
  CHECK(hit == true_hit);
  CHECK(rayhit.prim_id == prim_id);
  CHECK(rayhit.hit_distance == Approx(static_cast<T>(distance)));
  CHECK(rayhit.normal[0] == Approx(static_cast<T>(normal[0])));
  CHECK(rayhit.normal[1] == Approx(static_cast<T>(normal[1])));
  CHECK(rayhit.normal[2] == Approx(static_cast<T>(normal[2])));
}

#endif//BLAZERT_ASSERT_HELPER_H
