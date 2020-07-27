//
// Created by ogarten on 13/05/2020.
//

#ifndef BLAZERT_CYLINDER_H
#define BLAZERT_CYLINDER_H

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <blazert/bvh/options.h>
#include <blazert/datatypes.h>
#include <blazert/ray.h>

namespace blazert {

template<typename T>
class Cylinder {

public:
  const Vec3r<T> &center;
  const T &semi_axis_a;
  const T &semi_axis_b;
  const T &height;
  const Mat3r<T> &rotation;
  unsigned int prim_id;

public:
  Cylinder() = delete;
  Cylinder(const Vec3r<T> &center, const T &semi_axis_a, const T &semi_axis_b, const T &height,
           const Mat3r<T> &rotation, const unsigned int prim_id)
      : center(center), semi_axis_a(semi_axis_a), semi_axis_b(semi_axis_b), height(height), rotation(rotation),
        prim_id(prim_id){};
  Cylinder(Cylinder &&rhs) noexcept
      : center(std::move(rhs.center)), semi_axis_a(std::move(rhs.semi_axis_a)), semi_axis_b(std::move(rhs.semi_axis_b)),
        height(std::move(rhs.height)), rotation(std::move(rhs.rotation)), prim_id(std::exchange(rhs.prim_id, -1)) {}
  Cylinder &operator=(const Cylinder &rhs) = delete;
};

template<typename T, template<typename A> typename Collection,
         typename = std::enable_if_t<std::is_same<typename Collection<T>::primitive_type, Cylinder<T>>::value>>
inline Cylinder<T> primitive_from_collection(const Collection<T> &collection, const unsigned int prim_idx) {

  const Vec3r<T> &center = collection.centers[prim_idx];
  const T &semi_axis_a = collection.semi_axes_a[prim_idx];
  const T &semi_axis_b = collection.semi_axes_b[prim_idx];
  const T &height = collection.heights[prim_idx];
  const Mat3r<T> &rotation = collection.rotations[prim_idx];
  return {center, semi_axis_a, semi_axis_b, height, rotation, prim_idx};
}

template<typename T, template<typename A> typename Collection>
class CylinderIntersector {
public:
  const Collection<T> &collection;

  Vec3r<T> normal;

  Vec3r<T> ray_org;
  Vec3r<T> ray_dir;
  T min_hit_distance;
  Vec2r<T> uv;
  T hit_distance;
  unsigned int prim_id;

  CylinderIntersector() = delete;
  explicit CylinderIntersector(const Collection<T> &collection) : collection(collection), prim_id(static_cast<unsigned int>(-1)) {}
};

template<typename T>
class CylinderCollection {
public:
  typedef CylinderIntersector<T, CylinderCollection> intersector;
  typedef Cylinder<T> primitive_type;

  const Vec3rList<T> &centers;
  const std::vector<T> &semi_axes_a;
  const std::vector<T> &semi_axes_b;
  const std::vector<T> &heights;
  const Mat3rList<T> &rotations;

  std::vector<std::pair<Vec3r<T>, Vec3r<T>>> box;

public:
  CylinderCollection() = delete;
  CylinderCollection(const CylinderCollection &rhs) = delete;
  CylinderCollection(const Vec3rList<T> &centers, const std::vector<T> &semi_axis_a, const std::vector<T> &semi_axis_b,
                     const std::vector<T> &heights, const Mat3rList<T> &rotations)
      : centers(centers), semi_axes_a(semi_axis_a), semi_axes_b(semi_axis_b), heights(heights), rotations(rotations) {

    // pre calculate box
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
    return centers[prim_id];
  }

private:
  [[nodiscard]] inline std::pair<Vec3r<T>, Vec3r<T>>
  pre_compute_bounding_box(const unsigned int prim_id) const noexcept {
    const Vec3r<T> &center = centers[prim_id];
    const T a = semi_axes_a[prim_id];
    const T b = semi_axes_b[prim_id];
    const T height = heights[prim_id];
    const Mat3r<T> &rotation = rotations[prim_id];

    const Vec3r<T> &a1_tmp{a, 0, 0};
    const Vec3r<T> &b1_tmp{0, b, 0};
    const Vec3r<T> &h1_tmp{0, 0, static_cast<T>(height / 2.)};

    // These vectors describe the cylinder in the global coordinate system
    const Vec3r<T> &a1 = center + rotation * a1_tmp;
    const Vec3r<T> &b1 = center + rotation * b1_tmp;
    const Vec3r<T> &h1 = center + rotation * h1_tmp;

    const Vec3r<T> &a2 = center - rotation * a1_tmp;
    const Vec3r<T> &b2 = center - rotation * b1_tmp;
    const Vec3r<T> &h2 = center - rotation * h1_tmp;

    Vec3r<T> min{};
    Vec3r<T> max{};
    // maximum / minimum is also the max/min of the bounding box
    min[0] = std::min({a1[0], b1[0], a2[0], b2[0], h1[0], h2[0]});
    min[1] = std::min({a1[1], b1[1], a2[1], b2[1], h1[1], h2[1]});
    min[2] = std::min({a1[2], b1[2], a2[2], b2[2], h1[2], h2[2]});
    max[0] = std::max({a1[0], b1[0], a2[0], b2[0], h1[0], h2[0]});
    max[1] = std::max({a1[1], b1[1], a2[1], b2[1], h1[1], h2[1]});
    max[2] = std::max({a1[2], b1[2], a2[2], b2[2], h1[2], h2[2]});

    return std::make_pair(std::move(min), std::move(max));
  }
};

/**
 * Post BVH traversal stuff.
 * Fill `isect` if there is a hit.
 */
template<typename T, template<typename> typename Collection>
inline void post_traversal(CylinderIntersector<T, Collection> &i, RayHit<T> &rayhit) {
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
inline void prepare_traversal(CylinderIntersector<T, Collection> &i, const Ray<T> &ray) {
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
inline bool intersect_primitive(CylinderIntersector<T, Collection> &i, const Cylinder<T> &cylinder, const Ray<T> ray) {

  const Vec3r<T> &center = cylinder.center;
  const T semi_axis_a = cylinder.semi_axis_a;
  const T semi_axis_b = cylinder.semi_axis_b;
  const T height = cylinder.height;
  const Mat3r<T> &rotation = cylinder.rotation;

  const Mat3r<T> &inverse_rotation = trans(rotation);

  // center on coordinate system of object
  const Vec3r<T> &org = inverse_rotation * (i.ray_org - center);
  const Vec3r<T> &dir = inverse_rotation * i.ray_dir;

  // cylinder does not have to have a circle as base area, can also be ellipse
  // test in which area the source is
  /*
   *                 1
   *               -----..........
   *              |     |
   *              |  4  |
   *              |     |   3
   *              |     |
   *               -----..........
   *                 2
   *
   *
   */

  const T x0 = org[0];
  const T y0 = org[1];
  const T z0 = org[2];

  const T j = dir[0];
  const T k = dir[1];
  const T l = dir[2];

  const T a = semi_axis_a;
  const T b = semi_axis_b;
  const T h = height;

  /*
   * TODO: This legacy code which works...but is not nice at all. Needs refactoring!
   */
  // area 1
  if (z0 > h / static_cast<T>(2)) {
    if (l == 0)
      return false;
    const T t0 = (h / static_cast<T>(2) - z0) / l;
    if (t0 < 0)
      return false;

    const Vec3r<T> intercept = org + t0 * dir;

    // intercept point in circle
    if (b * b * intercept[0] * intercept[0] + a * a * intercept[1] * intercept[1] <= a * a * b * b) {
      const Vec3r<T> normal{0.f, 0.f, 1.f};
      const Vec3r<T> Ng = rotation * normal / norm(normal);
      i.normal = Ng;
      i.hit_distance = norm(t0 * dir);
      i.prim_id = cylinder.prim_id;
      return true;
    } else {
      const T A = a * a * k * k + b * b * j * j;
      const T B = a * a * k * y0 + b * b * j * x0;
      const T C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return false;

      const T t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const T t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<T> intercept0 = org + dir * t0;
      const Vec3r<T> intercept1 = org + dir * t1;

      // t1 is right intercept
      if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h / static_cast<T>(2))
          && (intercept1[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t1 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
      if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h / static_cast<T>(2))
          && (intercept0[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t0 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
    }
  }
  // area 2
  else if (z0 < -h / static_cast<T>(2)) {
    if (l == 0)
      return false;

    const T t0 = (-h / static_cast<T>(2) - z0) / l;
    if (t0 < 0)
      return false;

    const Vec3r<T> intercept = org + t0 * dir;

    // intercept point in circle
    if (b * b * intercept[0] * intercept[0] + a * a * intercept[1] * intercept[1] <= a * a * b * b) {
      const Vec3r<T> normal{0.f, 0.f, -1.f};
      const Vec3r<T> Ng = rotation * normal / norm(normal);
      i.normal = Ng;
      i.hit_distance = norm(t0 * dir);
      i.prim_id = cylinder.prim_id;
      return true;
    } else {
      const T A = a * a * k * k + b * b * j * j;
      const T B = a * a * k * y0 + b * b * j * x0;
      const T C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return false;

      const T t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const T t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<T> intercept0 = org + dir * t0;
      const Vec3r<T> intercept1 = org + dir * t1;
      // t1 is right intercept
      if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h / static_cast<T>(2))
          && (intercept1[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t1 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
      if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h / static_cast<T>(2))
          && (intercept0[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t0 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
    }
  }
  // area 3
  else if ((z0 > -h / static_cast<T>(2)) && (z0 < h / static_cast<T>(2))
           && (x0 * x0 * b * b + y0 * y0 * a * a > a * a * b * b)) {
    const T A = a * a * k * k + b * b * j * j;
    const T B = a * a * k * y0 + b * b * j * x0;
    const T C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

    if (C < 0)
      return false;
    const T t0 = 1.0 / A * (-B + a * b * sqrt(C));
    const T t1 = -1.0 / A * (B + a * b * sqrt(C));

    const Vec3r<T> intercept0 = org + dir * t0;
    const Vec3r<T> intercept1 = org + dir * t1;
    // t1 is right intercept
    if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h / static_cast<T>(2))
        && (intercept1[2] >= -h / static_cast<T>(2))) {
      const Vec3r<T> normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0.f};
      const Vec3r<T> Ng = rotation * normal / norm(normal);
      i.normal = Ng;
      i.hit_distance = norm(t1 * dir);
      i.prim_id = cylinder.prim_id;
      return true;
    }
    if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h / static_cast<T>(2))
        && (intercept0[2] >= -h / static_cast<T>(2))) {
      const Vec3r<T> normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0.f};
      const Vec3r<T> Ng = rotation * normal / norm(normal);
      i.normal = Ng;
      i.hit_distance = norm(t0 * dir);
      i.prim_id = cylinder.prim_id;
      return true;
    }
  }
  // area 4
  if ((z0 >= -h / static_cast<T>(2)) && (z0 <= h / static_cast<T>(2))
      && (x0 * x0 * b * b + y0 * y0 * a * a <= a * a * b * b)) {
    // only surrounding surface can have an intersection
    if (l == 0) {
      // if the z-component of the direction vector points along the
      // bottom or top circle, we will not have a intersection
      if ((z0 == -h / static_cast<T>(2)) || (z0 == h / static_cast<T>(2)))
        return false;

      const T A = a * a * k * k + b * b * j * j;
      const T B = a * a * k * y0 + b * b * j * x0;
      const T C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return false;

      const T t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const T t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<T> intercept0 = org + dir * t0;
      const Vec3r<T> intercept1 = org + dir * t1;
      // t1 is right intercept
      if ((t1 > 0) && (intercept1[2] <= h / static_cast<T>(2)) && (intercept1[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t1 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
      if ((t0 > 0) && (intercept0[2] <= h / static_cast<T>(2)) && (intercept0[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t0 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
    }
    // only happens if l != 0, an k, j == 0
    // only bottom or top can have intersection
    else if ((j == 0.f) && (k == 0.f)) {
      // if ray points along the circular surface, intresesction not
      // possible
      if (b * b * x0 * x0 + a * a * y0 * y0 == a * a * b * b)
        return false;
      // bottom circle
      if (l < 0) {
        const T t0 = (-h / static_cast<T>(2) - z0) / l;

        const Vec3r<T> normal{0.f, 0.f, -1.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t0 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
      // top circle
      if (l > 0) {
        const T t0 = (h / static_cast<T>(2) - z0) / l;

        const Vec3r<T> normal{0.f, 0.f, 1.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t0 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
    }
    // all cases where j, k and l are unequal to zero
    // all surfaces can have an intersection
    else {
      const T A = a * a * k * k + b * b * j * j;
      const T B = a * a * k * y0 + b * b * j * x0;
      const T C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return false;
      const T t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const T t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<T> intercept0 = org + dir * t0;
      const Vec3r<T> intercept1 = org + dir * t1;
      // t1 is right intercept

      if ((t1 > 0) && (intercept1[2] <= h / static_cast<T>(2)) && (intercept1[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t1 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
      if ((t0 > 0) && (intercept0[2] <= h / static_cast<T>(2)) && (intercept0[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0.f};
        i.normal = normal;
        i.hit_distance = norm(t0 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }

      // bottom circle
      if (l < 0) {
        const T t0 = (-h / static_cast<T>(2) - z0) / l;

        const Vec3r<T> normal{0.f, 0.f, -1.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t0 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
      // top circle
      if (l > 0) {
        const T t0 = (h / static_cast<T>(2) - z0) / l;

        const Vec3r<T> normal{0.f, 0.f, 1.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t0 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
    }
  }

  return false;
}

}// namespace blazert

#endif//BLAZERT_CYLINDER_H
