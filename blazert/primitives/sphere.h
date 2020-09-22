#pragma once
#ifndef BLAZERT_PRIMITIVES_SPHERE_H_
#define BLAZERT_PRIMITIVES_SPHERE_H_

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <blazert/bvh/options.h>
#include <blazert/datatypes.h>
#include <blazert/ray.h>

namespace blazert {

template<typename T>
class Sphere {

public:
  const Vec3r<T> &center;
  const T radius;
  unsigned int prim_id;

public:
  Sphere() = delete;
  Sphere(const Vec3r<T> &center, const T radius, const unsigned int prim_id)
      : center(center), radius(radius), prim_id(prim_id){};
  Sphere(Sphere &&rhs) noexcept
      : center(std::move(rhs.center)), radius(std::move(rhs.radius)), prim_id(std::exchange(rhs.prim_id, -1)) {}
  Sphere &operator=(const Sphere &rhs) = delete;
};

template<typename T>
std::ostream &operator<<(std::ostream &stream, const Sphere<T> &sphere) {
  /// Conveniently output a single plane as JSON.
  stream << "{\n";

  stream << "  Sphere: " << &sphere << ",\n";
  stream << "  center: [" << sphere.center[0] << "," << sphere.center[1] << "," << sphere.center[2] << "],\n";
  stream << "  radius: " << sphere.radius<< ",\n";
  stream << "  prim_id: " << sphere.prim_id << "\n";

  stream << "}\n";
  return stream;
}

template<typename T, template<typename A> typename Collection,
         typename = std::enable_if_t<std::is_same<typename Collection<T>::primitive_type, Sphere<T>>::value>>
[[nodiscard]] inline Sphere<T> primitive_from_collection(const Collection<T> &collection, const unsigned int prim_idx) {

  const Vec3r<T> &center = collection.centers[prim_idx];
  const T &radius = collection.radii[prim_idx];
  return {center, radius, prim_idx};
}

template<typename T, template<typename A> typename Collection>
class SphereIntersector {
public:
  const Collection<T> &collection;

  // ray_org and ray_dir needed for calculation of normal vector
  Vec3r<T> ray_org;
  Vec3r<T> ray_dir;
  T min_hit_distance;
  Vec2r<T> uv;
  T hit_distance;
  unsigned int prim_id;

  SphereIntersector() = delete;
  explicit SphereIntersector(const Collection<T> &collection)
      : collection(collection), prim_id(static_cast<unsigned int>(-1)) {}
};

template<typename T>
class SphereCollection {
public:
  typedef SphereIntersector<T, SphereCollection> intersector;
  typedef Sphere<T> primitive_type;

  const Vec3rList<T> &centers;
  const std::vector<T> &radii;
  std::vector<std::pair<Vec3r<T>, Vec3r<T>>> box;

  SphereCollection() = delete;
  SphereCollection(const SphereCollection<T> &rhs) = delete;
  SphereCollection(const Vec3rList<T> &centers, const std::vector<T> &radii) : centers(centers), radii(radii) {

    box.reserve(centers.size());

    for (unsigned int prim_id = 0; prim_id < centers.size(); prim_id++) {
      box.emplace_back(pre_compute_bounding_box(prim_id));
    }
  }

  [[nodiscard]] inline unsigned int size() const noexcept { return static_cast<unsigned int>(centers.size()); }

  [[nodiscard]] inline std::pair<Vec3r<T>, Vec3r<T>>
  get_primitive_bounding_box(const unsigned int prim_id) const noexcept {
    return box[prim_id];
  }

  [[nodiscard]] inline Vec3r<T> get_primitive_center(const unsigned int prim_id) const noexcept {
    return centers[prim_id];
  }

  [[nodiscard]] inline T distance_to_surface(const Vec3r<T> &point, const unsigned int prim_index) const noexcept {
    const Vec3r<T> &distance = centers[prim_index] - point;
    return abs(norm(distance) - radii[prim_index]);
  }

private:
  [[nodiscard]] inline std::pair<Vec3r<T>, Vec3r<T>>
  pre_compute_bounding_box(const unsigned int prim_id) const noexcept {

    Vec3r<T> min = centers[prim_id] - radii[prim_id];
    Vec3r<T> max = centers[prim_id] + radii[prim_id];

    return std::make_pair(std::move(min), std::move(max));
  }
};

template<typename T>
std::ostream &operator<<(std::ostream& stream, const SphereCollection<T> &collection) {
  stream << "{\n";
  stream << "SphereCollection: [\n";
  stream << "  size: " << collection.size() << ",\n";

  for(uint32_t id_sphere = 0; id_sphere < collection.size(); id_sphere++){
    stream << primitive_from_collection(collection, id_sphere);
    if(id_sphere == collection.size() - 1) {
      stream << "]\n";
    } else {
      stream << ", \n";
    }
  }

  stream << "}\n";
  return stream;
}

/**
   * Post BVH traversal stuff.
   * Fill `isect` if there is a hit.
   */
template<typename T, template<typename> typename Collection>
inline void post_traversal(SphereIntersector<T, Collection> &i, RayHit<T> &rayhit) {
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
inline void prepare_traversal(SphereIntersector<T, Collection> &i, const Ray<T> &ray) {
  i.ray_org = ray.origin;
  i.ray_dir = ray.direction;
  i.min_hit_distance = ray.min_hit_distance;
  i.hit_distance = ray.max_hit_distance;
  i.uv = static_cast<T>(0.);
  i.prim_id = static_cast<unsigned int>(-1);
}

/**
   * Do ray intersection stuff for `prim_index` th primitive and return hit
   * distance `hit_distance`, barycentric coordinate `u` and `v`.
   * Returns true if there's intersection.
   */
template<typename T, template<typename> typename Collection>
inline bool intersect_primitive(SphereIntersector<T, Collection> &i, const Sphere<T> &sphere,
                                [[maybe_unused]] const Ray<T> ray) {

  const Vec3r<T> &org = i.ray_org;
  const Vec3r<T> &dir = i.ray_dir;

  const Vec3r<T> &center = sphere.center;
  const T radius = sphere.radius;

  const Vec3r<T> &v = org - center;

  // calculate the actual intersections
  const T A = dot(dir, dir);
  const T B = static_cast<T>(2.0) * dot(v, dir);//
  const T C = dot(v, v) - radius * radius;      // distance to sphere
  const T D = B * B - static_cast<T>(4.0) * A * C;

  // interpretation of D: if D < 0 the ray misses the sphere and therefore
  // does not intersect term in the square root becomes negative
  if (D < static_cast<T>(0.0))
    return false;

  const T Q = std::sqrt(D);
  const T rcpA = static_cast<T>(1.) / A;
  const T t0 = static_cast<T>(0.5) * rcpA * (-B - Q);
  const T t1 = static_cast<T>(0.5) * rcpA * (-B + Q);

  // t0 corresponds to the first hit
  // hit distance is set to maximum in update_intersector
  if ((t0 > i.min_hit_distance) && (t0 < i.hit_distance)) {
    i.hit_distance = norm(t0 * dir);
    i.prim_id = sphere.prim_id;
    return true;
  }
  // t1 corresponds to the first hit
  if ((t1 > i.min_hit_distance) && (t1 < i.hit_distance)) {
    i.hit_distance = norm(t1 * dir);
    i.prim_id = sphere.prim_id;
    return true;
  }
  return false;
}

}// namespace blazert

#endif// BLAZERT_PRIMITIVES_SPHERE_H_
