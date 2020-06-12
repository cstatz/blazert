//
// Created by ogarten on 13/05/2020.
//

#ifndef BLAZERT_GEOM_TEMPLATE_H
#define BLAZERT_GEOM_TEMPLATE_H

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <blazert/bvh/options.h>
#include <blazert/datatypes.h>
#include <blazert/ray.h>

namespace blazert {

template<typename T>
class GEOM {

public:
  // TODO: Consitutive parameters of geometry

public:
  GEOM() = delete;
  GEOM(/* TODO: Consitutive parameters of geometry */ const unsigned int prim_id)
      : /* TODO: Consitutive parameters of geometry */ prim_id(prim_id){};
  GEOM(GEOM &&rhs) noexcept : /* TODO: Consitutive parameters of geometry */ prim_id(std::exchange(rhs.prim_id, -1)) {}
  GEOM &operator=(const GEOM &rhs) = delete;
};

template<typename T, template<typename A> typename Collection,
         typename = std::enable_if_t<std::is_same<typename Collection<T>::primitive_type, GEOM<T>>::value>>
inline GEOM<T> primitive_from_collection(const Collection<T> &collection, const unsigned int prim_idx) {

  /* TODO: grab consitutive parameters of geometry from collection */
  return {/* TODO: Consitutive parameters of geometry */ prim_idx};
};

template<typename T, template<typename A> typename Collection>
class GEOMIntersector {
public:
  const Collection<T> &collection;

  // ray_org and ray_dir needed for calculation of normal vector
  Vec3r<T> ray_org;
  Vec3r<T> ray_dir;
  T min_hit_distance;
  Vec2r<T> uv;
  T hit_distance;
  unsigned int prim_id;

  GEOMIntersector() = delete;
  explicit GEOMIntersector(const Collection<T> &collection) : collection(collection), prim_id(-1) {}
};

template<typename T>
class GEOMCollection {
public:
  typedef GEOMIntersector<T, GEOMCollection> intersector;
  typedef GEOM<T> primitive_type;

  /* TODO: containers for constitutive parameters of geometry */

  std::vector<std::pair<Vec3r<T>, Vec3r<T>>> box;

  GEOMCollection() = delete;
  GEOMCollection(const GEOMCollection<T> &rhs) = delete;
  GEOMCollection(/* TODO: containers for constitutive parameters of geometry */) /* TODO: initializer list */ {

    box.reserve(centers.size());

    for (unsigned int prim_id = 0; prim_id < centers.size(); prim_id++) {
      box.emplace_back(pre_compute_bounding_box(prim_id));
    }
  }

  [[nodiscard]] inline unsigned int size() const noexcept { return centers.size(); }

  [[nodiscard]] inline std::pair<Vec3r<T>, Vec3r<T>>
  get_primitive_bounding_box(const unsigned int prim_id) const noexcept {
    return box[prim_id];
  }

  [[nodiscard]] inline Vec3r<T> get_primitive_center(const unsigned int prim_id) const noexcept {
    // TODO: calculate center and return, for some primitives, center is constitutive parameter
    return centers[prim_id];
  }

private:
  [[nodiscard]] inline std::pair<Vec3r<T>, Vec3r<T>>
  pre_compute_bounding_box(const unsigned int prim_id) const noexcept {

    // TODO: Calculate bounding box
    return std::make_pair(std::move(min), std::move(max));
  }
};

/**
   * Post BVH traversal stuff.
   * Fill `isect` if there is a hit.
   */
template<typename T, template<typename> typename Collection>
inline void post_traversal(GEOMIntersector<T, Collection> &i, RayHit<T> &rayhit) {
  rayhit.hit_distance = i.hit_distance;
  rayhit.uv = i.uv;
  rayhit.prim_id = i.prim_id;
  const Vec3r<T> &center = i.collection.get_primitive_center(i.prim_id);
  rayhit.normal = normalize(i.ray_org + i.hit_distance * i.ray_dir - center);
}

/**
   * Prepare BVH traversal (e.g. compute inverse ray direction).
   * This function is called only once in BVH traversal.
   */
template<typename T, template<typename> typename Collection>
inline void prepare_traversal(GEOMIntersector<T, Collection> &i, const Ray<T> &ray) {
  i.ray_org = ray.origin;
  i.ray_dir = ray.direction;
  i.min_hit_distance = ray.min_hit_distance;
  i.hit_distance = ray.max_hit_distance;
  i.uv = static_cast<T>(0.);
  i.prim_id = -1;
}

/**
   * Do ray intersection stuff for `prim_index` th primitive and return hit
   * distance `hit_distance`, barycentric coordinate `u` and `v`.
   * Returns true if there's intersection.
   */
template<typename T, template<typename> typename Collection>
inline bool intersect_primitive(GEOMIntersector<T, Collection> &i, const GEOM<T> &GEOM, const Ray<T> ray) {
  /* TODO: Calculate intersections */
}

}// namespace blazert

#endif//BLAZERT_GEOM_TEMPLATE_H
