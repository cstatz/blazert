//
// Created by ogarten on 09/06/2020.
//

#ifndef BLAZERT_ASSERT_HELPER_H
#define BLAZERT_ASSERT_HELPER_H

#include <blazert/bvh/accel.h>
#include <blazert/bvh/builder.h>
#include <blazert/datatypes.h>
#include <blazert/ray.h>
#include <third_party/doctest/doctest/doctest.h>

using namespace blazert;
using namespace doctest;

template<typename T, template<typename> typename Collection>
void test_intersect_primitive_hit(const Collection<T> &collection, const Ray<T> &ray, const unsigned int prim_id, const T distance, const Vec3r<T> &normal) {
  typename Collection<T>::intersector intersector(collection);
  RayHit<T> rayhit;
  // Test intersections
  prepare_traversal(intersector, ray);
  const bool hit = intersect_primitive(intersector, primitive_from_collection(collection, 0), ray);
  post_traversal(intersector, rayhit);

  CHECK(hit);
  CHECK(rayhit.prim_id == prim_id);
  CHECK(rayhit.hit_distance == Approx(static_cast<T>(distance)));
  CHECK(rayhit.normal[0] == Approx(static_cast<T>(normal[0])));
  CHECK(rayhit.normal[1] == Approx(static_cast<T>(normal[1])));
  CHECK(rayhit.normal[2] == Approx(static_cast<T>(normal[2])));
}

template<typename T, template<typename> typename Collection>
void test_intersect_primitive_no_hit(const Collection<T> &collection, const Ray<T> &ray) {
  typename Collection<T>::intersector intersector(collection);
  RayHit<T> rayhit;
  // Test intersections
  prepare_traversal(intersector, ray);
  const bool hit = intersect_primitive(intersector, primitive_from_collection(collection, 0), ray);
  post_traversal(intersector, rayhit);

  CHECK_FALSE(hit);
}

template<typename T, template<typename> typename Collection>
void test_traverse_bvh_hit(const Collection<T> &collection, const Ray<T> &ray, const unsigned int prim_id, const T distance, const Vec3r<T> &normal) {

  BVH bvh(collection);
  SAHBinnedBuilder builder;

  auto statistics = builder.build(bvh);

  RayHit<T> rayhit{};
  const bool hit = traverse(bvh, ray, rayhit);
  CHECK(hit);
  CHECK(rayhit.prim_id == prim_id);
  CHECK(rayhit.hit_distance == Approx(static_cast<T>(distance)));
  CHECK(rayhit.normal[0] == Approx(static_cast<T>(normal[0])));
  CHECK(rayhit.normal[1] == Approx(static_cast<T>(normal[1])));
  CHECK(rayhit.normal[2] == Approx(static_cast<T>(normal[2])));
}

template<typename T, template<typename> typename Collection>
void test_traverse_bvh_no_hit(const Collection<T> &collection, const Ray<T> &ray) {

  BVH bvh(collection);
  SAHBinnedBuilder builder;

  auto statistics = builder.build(bvh);

  RayHit<T> rayhit{};
  const bool hit = traverse(bvh, ray, rayhit);
  CHECK_FALSE(hit);
}

#endif//BLAZERT_ASSERT_HELPER_H
