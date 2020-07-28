//
// Created by ogarten on 13/05/2020.
//

#ifndef BLAZERT_PLANE_H
#define BLAZERT_PLANE_H

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <blazert/bvh/options.h>
#include <blazert/datatypes.h>
#include <blazert/ray.h>

namespace blazert {

template<typename T>
class Plane {

public:
  const Vec3r<T> &center;
  const T dx;
  const T dy;
  const Mat3r<T> &rotation;
  unsigned int prim_id;

  constexpr static T thickness = std::numeric_limits<T>::min();

public:
  Plane() = delete;
  Plane(const Vec3r<T> &center, const T dx, const T dy, const Mat3r<T> &rotation, const unsigned int prim_id)
      : center(center), dx(dx), dy(dy), rotation(rotation), prim_id{prim_id} {};
  Plane(Plane &&rhs) noexcept
      : center(std::move(rhs.center)), dx(std::move(rhs.dx)), dy(std::move(rhs.dy)), rotation(std::move(rhs.rotation)),
        prim_id(std::exchange(rhs.prim_id, -1)) {}
  Plane &operator=(const Plane &rhs) = delete;
};

template<typename T, template<typename A> typename Collection,
         typename = std::enable_if_t<std::is_same<typename Collection<T>::primitive_type, Plane<T>>::value>>
[[nodiscard]] inline Plane<T> primitive_from_collection(const Collection<T> &collection, const unsigned int prim_idx) {

  const Vec3r<T> &center = collection.centers[prim_idx];
  const T &dx = collection.dxs[prim_idx];
  const T &dy = collection.dys[prim_idx];
  const Mat3r<T> &rotation = collection.rotations[prim_idx];
  return {center, dx, dy, rotation, prim_idx};
}

template<typename T, template<typename A> typename Collection>
class PlaneIntersector {
public:
  const Collection<T> &collection;

  Vec3r<T> normal;

  Vec3r<T> ray_org;
  Vec3r<T> ray_dir;
  T min_hit_distance;
  Vec2r<T> uv;
  T hit_distance;
  unsigned int prim_id;

  PlaneIntersector() = delete;
  explicit PlaneIntersector(const Collection<T> &collection)
      : collection(collection), prim_id(static_cast<unsigned int>(-1)) {}
};

template<typename T>
class PlaneCollection {
public:
  typedef PlaneIntersector<T, PlaneCollection> intersector;
  typedef Plane<T> primitive_type;

  const Vec3rList<T> &centers;
  const std::vector<T> &dxs;
  const std::vector<T> &dys;
  const Mat3rList<T> &rotations;

  constexpr static T thickness = std::numeric_limits<T>::min();

  std::vector<std::pair<Vec3r<T>, Vec3r<T>>> box;

public:
  PlaneCollection() = delete;
  PlaneCollection(const PlaneCollection &rhs) = delete;
  PlaneCollection(const Vec3rList<T> &centers, const std::vector<T> &dxs, const std::vector<T> &dys,
                  const Mat3rList<T> &rotations)
      : centers(centers), dxs(dxs), dys(dys), rotations(rotations) {

    // pre calculate box
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

private:
  [[nodiscard]] inline std::pair<Vec3r<T>, Vec3r<T>>
  pre_compute_bounding_box(const unsigned int prim_id) const noexcept {
    const Vec3r<T> &center = centers[prim_id];
    const T dx = dxs[prim_id];
    const T dy = dys[prim_id];
    const Mat3r<T> &rotation = rotations[prim_id];

    // vectors describing extent in x direction
    const Vec3r<T> &a1_tmp{-dx / static_cast<T>(2.0), -dy / static_cast<T>(2.0), 0.0};
    const Vec3r<T> &a2_tmp{dx / static_cast<T>(2.0), -dy / static_cast<T>(2.0), 0.0};
    // vectors describing extent in y direction
    const Vec3r<T> &a3_tmp{-dx / static_cast<T>(2.0), dy / static_cast<T>(2.0), 0.0};
    const Vec3r<T> &a4_tmp{dx / static_cast<T>(2.0), dy / static_cast<T>(2.0), 0.0};

    // vectors describing extent in y direction
    const Vec3r<T> &c1_tmp{0.0, 0.0, -thickness};
    const Vec3r<T> &c2_tmp{0.0, 0.0, thickness};

    // vectors describing the plane in the global coordinate system
    const Vec3r<T> &a1 = center + rotation * a1_tmp;
    const Vec3r<T> &a2 = center + rotation * a2_tmp;
    const Vec3r<T> &a3 = center + rotation * a3_tmp;
    const Vec3r<T> &a4 = center + rotation * a4_tmp;
    const Vec3r<T> &c1 = center + rotation * c1_tmp;
    const Vec3r<T> &c2 = center + rotation * c2_tmp;

    // maximum / minimum is also the max/min of the bounding box
    Vec3r<T> min{};
    Vec3r<T> max{};
    min[0] = std::min({a1[0], a2[0], a3[0], a4[0], c1[0], c2[0]});
    min[1] = std::min({a1[1], a2[1], a3[1], a4[1], c1[1], c2[1]});
    min[2] = std::min({a1[2], a2[2], a3[2], a4[2], c1[2], c2[2]});
    max[0] = std::max({a1[0], a2[0], a3[0], a4[0], c1[0], c2[0]});
    max[1] = std::max({a1[1], a2[1], a3[1], a4[1], c1[1], c2[1]});
    max[2] = std::max({a1[2], a2[2], a3[2], a4[2], c1[2], c2[2]});

    return std::make_pair(std::move(min), std::move(max));
  }
};

template<typename T, template<typename> typename Collection>
inline void post_traversal(PlaneIntersector<T, Collection> &i, RayHit<T> &rayhit) {
  rayhit.hit_distance = i.hit_distance;
  rayhit.uv = i.uv;
  rayhit.prim_id = i.prim_id;
  rayhit.normal = normalize(i.normal);
}

/**
   * Prepare BVH traversal (e.g. compute inverse ray direction).
   * This function is called only once in BVH traversal.
   */
template<typename T, template<typename> typename Collection>
inline void prepare_traversal(PlaneIntersector<T, Collection> &i, const Ray<T> &ray) {
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
inline bool intersect_primitive(PlaneIntersector<T, Collection> &i, const Plane<T> &plane,
                                [[maybe_unused]] const Ray<T> ray) {

  const Vec3r<T> &center = plane.center;
  const T dx = plane.dx;
  const T dy = plane.dy;
  const Mat3r<T> &rotation = plane.rotation;
  const Mat3r<T> &inverse_rotation = trans(rotation);

  const Vec3r<T> &org = inverse_rotation * (i.ray_org - center);
  const Vec3r<T> &dir = inverse_rotation * i.ray_dir;

  // calculate interception point
  const T t1 = -org[2] / dir[2];
  const Vec3r<T> &intercept = org + t1 * dir;

  const T tnear = i.min_hit_distance;
  const T tfar = i.hit_distance;

  const T x_min = -dx / 2;
  const T x_max = dx / 2;

  const T y_min = -dy / 2;
  const T y_max = dy / 2;

  // is intercept within ray limits?
  // does it actually hit the plane?
  if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] < x_max) && (intercept[1] > y_min)
      && (intercept[1] < y_max)) {
    i.normal = org[2] / abs(org[2]) * (rotation * Vec3r<T>{0.0, 0.0, 1.0});
    i.hit_distance = norm(t1 * dir);
    i.prim_id = plane.prim_id;
    return true;
  }

  // plane edges
  else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] == x_min) && (intercept[0] < x_max) && (intercept[1] > y_min)
           && (intercept[1] < y_max)) {
    i.normal = rotation * Vec3r<T>{-1.0, 0.0, org[2] / abs(org[2])};
    i.hit_distance = norm(t1 * dir);
    i.prim_id = plane.prim_id;
    return true;
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] == x_max) && (intercept[1] > y_min)
             && (intercept[1] < y_max)) {
    i.normal = rotation * Vec3r<T>{1.0, 0.0, org[2] / abs(org[2])};
    i.hit_distance = norm(t1 * dir);
    i.prim_id = plane.prim_id;
    return true;
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] < x_max) && (intercept[1] == y_min)
             && (intercept[1] < y_max)) {
    i.normal = rotation * Vec3r<T>{0.0, -1.0, org[2] / abs(org[2])};
    i.hit_distance = norm(t1 * dir);
    i.prim_id = plane.prim_id;
    return true;
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] < x_max) && (intercept[1] > y_min)
             && (intercept[1] == y_max)) {
    i.normal = rotation * Vec3r<T>{0.0, 1.0, org[2] / abs(org[2])};
    i.hit_distance = norm(t1 * dir);
    i.prim_id = plane.prim_id;
    return true;
  }

  // plane corners
  else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] == x_min) && (intercept[0] < x_max) && (intercept[1] == y_min)
           && (intercept[1] < y_max)) {
    i.normal = rotation * Vec3r<T>{-1.0, -1.0, org[2] / abs(org[2])};
    i.hit_distance = norm(t1 * dir);
    i.prim_id = plane.prim_id;
    return true;
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] == x_min) && (intercept[0] < x_max) && (intercept[1] > y_min)
             && (intercept[1] == y_max)) {
    i.normal = rotation * Vec3r<T>{-1.0, 1.0, org[2] / abs(org[2])};
    i.hit_distance = norm(t1 * dir);
    i.prim_id = plane.prim_id;
    return true;
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] == x_max) && (intercept[1] == y_min)
             && (intercept[1] < y_max)) {
    i.normal = rotation * Vec3r<T>{1.0, -1.0, org[2] / abs(org[2])};
    i.hit_distance = norm(t1 * dir);
    i.prim_id = plane.prim_id;
    return true;
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] == x_max) && (intercept[1] > y_min)
             && (intercept[1] == y_max)) {
    i.normal = rotation * Vec3r<T>{1.0, 1.0, org[2] / abs(org[2])};
    i.hit_distance = norm(t1 * dir);
    i.prim_id = plane.prim_id;
    return true;
  }
  return false;
}

}// namespace blazert

#endif//BLAZERT_PLANE_H
