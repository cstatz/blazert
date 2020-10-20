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

/***
 * Asserts all necessary boundaries for the primitive with prim_id within the collection.
 * @tparam T primitive type
 * @tparam Collection collection type
 *
 * @param collection a collection of primitives
 * @param prim_id unique id for a certain primitive within the collection
 * @param true_min minimum point of the bounding box to assert against
 * @param true_max maximum point of the bounding box to assert against
 */
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

/***
 * Asserts whether the center of the primitive with prim_id from collection is at true_center.
 *
 * @tparam T primitive type
 * @tparam Collection collection type
 *
 * @param collection a collection of primitives
 * @param prim_id unique id for a certain primitive within the collection
 * @param true_center center location to assert against
 */
template<typename T, template<typename> typename Collection>
inline void assert_primitive_center(const Collection<T> &collection, const unsigned int prim_id,
                                    const Vec3r<T> &true_center) {
  const auto center = collection.get_primitive_center(prim_id);

  CHECK(center[0] == Approx(true_center[0]));
  CHECK(center[1] == Approx(true_center[1]));
  CHECK(center[2] == Approx(true_center[2]));
}

/***
 * Asserts whether the distance from an arbitrary point to the surface of the primitive with prim_id from collection
 * is equal to true_distance
 *
 * @tparam T primitive type
 * @tparam Collection collection type
 *
 * @param collection a collection of primitives
 * @param prim_id unique id for a certain primitive within the collection
 * @param point point from which the distance is to be calculated
 * @param true_distance distance to asssert against
 */
template<typename T, template<typename> typename Collection>
inline void assert_distance_to_surface(const Collection<T> &collection, const unsigned int prim_id,
                                       const Vec3r<T> &point, const T true_distance) {
  CHECK(collection.distance_to_surface(point, prim_id) == Approx(true_distance));
}

/***
 * Asserts whether a ray (origin, direction) hits a primitive with prim_id from the collection. It also asserts if the
 * computed results (distance, normal vector) are correct.
 *
 * @tparam T primitive type
 * @tparam Collection collection type
 *
 * @param collection a collection of primitives
 * @param ray Ray against for which the intersection is computed
 * @param true_hit true if a hit is expected
 * @param prim_id unique id for a certain primitive within the collection
 * @param distance expected distance between ray origin and the intersection point
 * @param normal expected normal vector on the surface of the primitive at the intersection point
 */
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
 * Asserts whether a ray hits the primitive with prim_id from the collection by traversing the BVH. It also asserts if
 * the computed results (distance, normal vector) are correct.
 *
 * @tparam T primitive type
 * @tparam Collection collection type
 *
 * @param collection a collection of primitives
 * @param ray Ray against for which the intersection is computed
 * @param true_hit true if a hit is expected
 * @param prim_id unique id for a certain primitive within the collection
 * @param distance expected distance between ray origin and the intersection point
 * @param normal expected normal vector on the surface of the primitive at the intersection point
 */
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
