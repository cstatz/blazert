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
[[nodiscard]] inline Cylinder<T> primitive_from_collection(const Collection<T> &collection,
                                                           const unsigned int prim_idx) {

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
  explicit CylinderIntersector(const Collection<T> &collection)
      : collection(collection), prim_id(static_cast<unsigned int>(-1)) {}
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
    const T a = semi_axes_a[prim_id];
    const T b = semi_axes_b[prim_id];
    const T height = heights[prim_id];
    const Mat3r<T> &rotation = rotations[prim_id];

    const Vec3r<T> &a1_tmp{a, 0, 0};
    const Vec3r<T> &b1_tmp{0, b, 0};
    const Vec3r<T> &h1_tmp{0, 0, height / static_cast<T>(2.)};

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
inline bool intersect_primitive(CylinderIntersector<T, Collection> &i, const Cylinder<T> &cylinder,
                                [[maybe_unused]] const Ray<T> ray) {

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
      const Vec3r<T> normal{0, 0, 1};
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

      const T t0 = static_cast<T>(1.0f) / A * (-B + a * b * std::sqrt(C));
      const T t1 = static_cast<T>(-1.0) / A * (B + a * b * std::sqrt(C));

      const Vec3r<T> intercept0 = org + dir * t0;
      const Vec3r<T> intercept1 = org + dir * t1;

      // t1 is right intercept
      if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h / static_cast<T>(2))
          && (intercept1[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t1 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
      if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h / static_cast<T>(2))
          && (intercept0[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0};
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
      const Vec3r<T> normal{0, 0, -1};
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

      const T t0 = static_cast<T>(1.0) / A * (-B + a * b * std::sqrt(C));
      const T t1 = static_cast<T>(-1.0) / A * (B + a * b * std::sqrt(C));

      const Vec3r<T> intercept0 = org + dir * t0;
      const Vec3r<T> intercept1 = org + dir * t1;
      // t1 is right intercept
      if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h / static_cast<T>(2))
          && (intercept1[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t1 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
      if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h / static_cast<T>(2))
          && (intercept0[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0};
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
    const T t0 = static_cast<T>(1.0) / A * (-B + a * b * std::sqrt(C));
    const T t1 = static_cast<T>(-1.0) / A * (B + a * b * std::sqrt(C));

    const Vec3r<T> intercept0 = org + dir * t0;
    const Vec3r<T> intercept1 = org + dir * t1;
    // t1 is right intercept
    if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h / static_cast<T>(2))
        && (intercept1[2] >= -h / static_cast<T>(2))) {
      const Vec3r<T> normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0};
      const Vec3r<T> Ng = rotation * normal / norm(normal);
      i.normal = Ng;
      i.hit_distance = norm(t1 * dir);
      i.prim_id = cylinder.prim_id;
      return true;
    }
    if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h / static_cast<T>(2))
        && (intercept0[2] >= -h / static_cast<T>(2))) {
      const Vec3r<T> normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0};
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

      const T t0 = static_cast<T>(1.0) / A * (-B + a * b * std::sqrt(C));
      const T t1 = static_cast<T>(-1.0) / A * (B + a * b * std::sqrt(C));

      const Vec3r<T> intercept0 = org + dir * t0;
      const Vec3r<T> intercept1 = org + dir * t1;
      // t1 is right intercept
      if ((t1 > 0) && (intercept1[2] <= h / static_cast<T>(2)) && (intercept1[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t1 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
      if ((t0 > 0) && (intercept0[2] <= h / static_cast<T>(2)) && (intercept0[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t0 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
    }
    // only happens if l != 0, an k, j == 0
    // only bottom or top can have intersection
    else if ((j == static_cast<T>(0.)) && (k == static_cast<T>(0))) {
      // if ray points along the circular surface, intresesction not
      // possible
      if (b * b * x0 * x0 + a * a * y0 * y0 == a * a * b * b)
        return false;
      // bottom circle
      if (l < 0) {
        const T t0 = (-h / static_cast<T>(2) - z0) / l;

        const Vec3r<T> normal{0, 0, -1};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t0 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
      // top circle
      if (l > 0) {
        const T t0 = (h / static_cast<T>(2) - z0) / l;

        const Vec3r<T> normal{0, 0, 1};
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
      const T t0 = static_cast<T>(1.0) / A * (-B + a * b * std::sqrt(C));
      const T t1 = static_cast<T>(-1.0) / A * (B + a * b * std::sqrt(C));

      const Vec3r<T> intercept0 = org + dir * t0;
      const Vec3r<T> intercept1 = org + dir * t1;
      // t1 is right intercept

      if ((t1 > 0) && (intercept1[2] <= h / static_cast<T>(2)) && (intercept1[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept1[0] / (a * a), 2 * intercept1[1] / (b * b), 0};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t1 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
      if ((t0 > 0) && (intercept0[2] <= h / static_cast<T>(2)) && (intercept0[2] >= -h / static_cast<T>(2))) {
        const Vec3r<T> normal{2 * intercept0[0] / (a * a), 2 * intercept0[1] / (b * b), 0};
        i.normal = normal;
        i.hit_distance = norm(t0 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }

      // bottom circle
      if (l < 0) {
        const T t0 = (-h / static_cast<T>(2) - z0) / l;

        const Vec3r<T> normal{0, 0, -1};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        i.hit_distance = norm(t0 * dir);
        i.prim_id = cylinder.prim_id;
        return true;
      }
      // top circle
      if (l > 0) {
        const T t0 = (h / static_cast<T>(2) - z0) / l;

        const Vec3r<T> normal{0, 0, 1};
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

namespace {

/**
 * This is the ellipse equation. If return value is smaller than 0, 'point' is inside, if larger than 0, point is outside.
 * @tparam T
 * @param a
 * @param b
 * @param point
 * @return
 */
template<typename T>
inline T ellipse_equation(const T a, const T b, const Vec2r<T> &point) {
  const T x = point[0];
  const T y = point[1];
  return (x * x) / (a * a) + (y * y) / (b * b) - 1;
}

enum class HALFSPACE { UPPER, LOWER };
enum class ARGUMENT { X, Y };

template<typename T, HALFSPACE hs, ARGUMENT arg>
inline T distance_derivative(const T a, const T b, const Vec2r<T> &point, const T val) {
  if constexpr ((hs == HALFSPACE::UPPER) && (arg == ARGUMENT::X)) {
    const T sqrt = (val * val > a * a) ? std::sqrt(val * val - a * a) : std::sqrt(a * a - val * val);
    const T xq = point[0];
    const T yq = point[1];

    return -2 * (xq - val) - 2 * b * val * (b / a * sqrt - yq) / (a * sqrt);
  } else if constexpr ((hs == HALFSPACE::LOWER) && (arg == ARGUMENT::X)) {
    const T sqrt = (val * val > a * a) ? std::sqrt(val * val - a * a) : std::sqrt(a * a - val * val);
    const T xq = point[0];
    const T yq = point[1];

    return -2 * (xq - val) - 2 * b * val * (b / a * sqrt + yq) / (a * sqrt);
  } else if constexpr ((hs == HALFSPACE::UPPER) && (arg == ARGUMENT::Y)) {
    const T sqrt = (val * val > b * b) ? std::sqrt(val * val - b * b) : std::sqrt(b * b - val * val);
    const T xq = point[0];
    const T yq = point[1];

    return -2 * (yq - val) - 2 * a * val * (a / b * sqrt + xq) / (b * sqrt);
  } else if constexpr ((hs == HALFSPACE::LOWER) && (arg == ARGUMENT::Y)) {
    const T sqrt = (val * val > b * b) ? std::sqrt(val * val - b * b) : std::sqrt(b * b - val * val);
    const T xq = point[0];
    const T yq = point[1];

    return -2 * (yq - val) - 2 * a * val * (a / b * sqrt - xq) / (b * sqrt);
  }
}

// formula for a single iteration step in the Newton-Raphson algorithm for 4 cases.
// returns x - R'(x)/R''(x)
template<typename T, HALFSPACE hs, ARGUMENT arg>
inline T single_newton_iteration(const T a, const T b, const Vec2r<T> &point, const T val) {

  if constexpr ((hs == HALFSPACE::UPPER) && (arg == ARGUMENT::X)) {
    const T sqrt = (val * val > a * a) ? std::sqrt(val * val - a * a) : std::sqrt(a * a - val * val);
    const T xq = point[0];
    const T yq = point[1];

    return ((a * a * a * xq - a * val * val * xq) * sqrt + b * val * val * val * yq)
        / ((a * a * a - a * b * b - a * val * val + b * b * val * val / a) * sqrt + a * a * b * yq);
  }
  if constexpr ((hs == HALFSPACE::LOWER) && (arg == ARGUMENT::X)) {
    const T sqrt = (val * val > a * a) ? std::sqrt(val * val - a * a) : std::sqrt(a * a - val * val);
    const T xq = point[0];
    const T yq = point[1];

    return ((-a * a * a * xq + a * val * val * xq) * sqrt + b * val * val * val * yq)
        / ((-a * a * a + a * b * b + a * val * val - b * b * val * val / a) * sqrt + a * a * b * yq);
  }
  if constexpr ((hs == HALFSPACE::UPPER) && (arg == ARGUMENT::Y)) {
    const T sqrt = (val * val > b * b) ? std::sqrt(val * val - b * b) : std::sqrt(b * b - val * val);
    const T xq = point[0];
    const T yq = point[1];

    return ((b * b * b * yq - b * val * val * yq) * sqrt + a * val * val * val * xq)
        / ((b * b * b - b * a * a - b * val * val + a * a * val * val / b) * sqrt + b * b * a * xq);
  }
  if constexpr ((hs == HALFSPACE::LOWER) && (arg == ARGUMENT::Y)) {
    const T sqrt = (val * val > b * b) ? std::sqrt(val * val - b * b) : std::sqrt(b * b - val * val);
    const T xq = point[0];
    const T yq = point[1];

    return ((-b * b * b * yq + b * val * val * yq) * sqrt + a * val * val * val * xq)
        / ((-b * b * b + b * a * a + b * val * val - a * a * val * val / b) * sqrt + b * b * a * xq);
  }
}

// Newton iteration for given function
template<typename T, typename single_iteration_fcn, typename stop_criterion>
inline T newton_iteration(const T a, const T b, const Vec2r<T> &point, const T initial_guess,
                          single_iteration_fcn &single_iteration, stop_criterion &stop_criterion_fcn) {
  T val0 = initial_guess;
  T cnt = 0;
  constexpr T epsilon = []() constexpr->T {
    if constexpr (std::is_same_v<T, float>) {
      return static_cast<T>(1e-4);
    } else {
      return static_cast<T>(1e-7);
    }
  }
  ();
  while ((std::abs(stop_criterion_fcn(a, b, point, val0)) >= epsilon) && (cnt < 100)) {
    const T val = single_iteration(a, b, point, val0);
    val0 = val;
    cnt++;
    if (std::abs(val0) == a)
      val0 += std::numeric_limits<T>::epsilon();
  }
  return val0;
}

/**
 * @brief Newton-Raphson root finding algorithm to find the minimal radius of a circle aroung 'xq' and 'yq'
 *
 * @tparam T floating point type
 * @return returns std::pair<Vec2r<T>, T> (closest_point, shortest distance)
 */
template<typename T>
inline std::pair<Vec2r<T>, T> find_minimum_distance_to_ellipse(const T a, const T b, const Vec2r<T> &point) {
  const T xq = point[0];
  const T yq = point[1];

  constexpr T percentage = static_cast<T>(0.95);

  if (((xq >= 0) && (yq >= (b / a) * xq)) || ((xq <= 0) && (yq >= (-b / a) * xq))) {
    const T x0 = xq > 0 ? percentage * a : -percentage * a;
    const T x_min = newton_iteration(a, b, point, x0, single_newton_iteration<T, HALFSPACE::UPPER, ARGUMENT::X>,
                                     distance_derivative<T, HALFSPACE::UPPER, ARGUMENT::X>);
    const T sqrt = std::sqrt(a * a - x_min * x_min);

    const T y_min = b / a * sqrt;

    const T min_dist = std::sqrt((xq - x_min) * (xq - x_min) + (yq - y_min) * (yq - y_min));
    return {Vec2r<T>{x_min, y_min}, min_dist};
  } else if (((xq >= 0) && (yq <= (-b / a) * xq)) || ((xq <= 0) && (yq <= (b / a) * xq))) {
    const T x0 = xq > 0 ? percentage * a : -percentage * a;
    const T x_min = newton_iteration(a, b, point, x0, single_newton_iteration<T, HALFSPACE::LOWER, ARGUMENT::X>,
                                     distance_derivative<T, HALFSPACE::LOWER, ARGUMENT::X>);
    const T sqrt = std::sqrt(a * a - x_min * x_min);

    const T y_min = -b / a * sqrt;

    const T min_dist = std::sqrt((xq - x_min) * (xq - x_min) + (yq - y_min) * (yq - y_min));
    return {Vec2r<T>{x_min, y_min}, min_dist};
  } else if ((xq >= 0) && (yq <= (b / a) * xq) && (yq >= (-b / a) * xq)) {
    const T y0 = yq > 0 ? percentage * b : -percentage * b;

    const T y_min = newton_iteration(a, b, point, y0, single_newton_iteration<T, HALFSPACE::UPPER, ARGUMENT::Y>,
                                     distance_derivative<T, HALFSPACE::UPPER, ARGUMENT::Y>);

    const T sqrt = std::sqrt(b * b - y_min * y_min);
    const T x_min = a / b * sqrt;

    const T min_dist = std::sqrt((yq - y_min) * (yq - y_min) + (xq - x_min) * (xq - x_min));
    return {Vec2r<T>{x_min, y_min}, min_dist};
  } else {//if ((xq <= 0) && (yq <= (-b / a) * xq) && (yq >= (b / a) * xq)) {
    const T y0 = yq > 0 ? percentage * b : -percentage * b;
    const T y_min = newton_iteration(a, b, point, y0, single_newton_iteration<T, HALFSPACE::LOWER, ARGUMENT::Y>,
                                     distance_derivative<T, HALFSPACE::LOWER, ARGUMENT::Y>);

    const T sqrt = std::sqrt(b * b - y_min * y_min);
    const T x_min = -a / b * sqrt;

    const T min_dist = std::sqrt((yq - y_min) * (yq - y_min) + (xq - x_min) * (xq - x_min));
    return {Vec2r<T>{x_min, y_min}, min_dist};
  }
}

}// namespace
/**
 * @brief The distance to surface function returns the distance from a query point to the surface of the cylinder.
 *
 * @details
 *
 * @tparam T
 * @param cylinder
 * @param point
 * @return
 */
template<typename T>
[[nodiscard]] inline T distance_to_surface(const Cylinder<T> &cylinder, const Vec3r<T> &point) noexcept {

  //std::cout << "==========================\n"
  const Vec3r<T> &local_point = trans(cylinder.rotation) * (point - cylinder.center);

  // 0. calculate distance to top
  const T dist_top = std::fabs(local_point[2] - cylinder.height / 2);

  // 1. calculate distance to bottom
  const T dist_bottom = std::fabs(local_point[2] + cylinder.height / 2);

  // idea:
  // - distance is radius of smallest sphere around 'point'
  // - for ellipse, we have smallest circle in a plane parallel to z
  // - combine circle equation and ellipse equation to calculate d^2 (two solutions)
  //    - use derivative to find minimum -> Newton-Raphson Iteration
  //    - iterate until some threshold is reached (e.g. relative change, number of iterations, ...)
  //    - use solution to calculate all 4 'd' and take minimum

  const T x_q = local_point[0];
  const T y_q = local_point[1];
  const T z_q = local_point[2];

  const T a = cylinder.semi_axis_a;
  const T b = cylinder.semi_axis_b;

  const Vec2r<T> point_xy{x_q, y_q};

  // if the points is in the center, the distance is determined by how far away each of the shells is
  if (isZero(point_xy)) {
    if ((z_q > cylinder.height / 2) || (z_q < -cylinder.height / 2)) {
      return std::min({dist_top, dist_bottom});
    } else {
      return std::min({a, b, dist_top, dist_bottom});
    }
  }

  // if the query point is directly on the ellipse, the will iteration will not converge
  const T ellipse = ellipse_equation(a, b, point_xy);

  // if point is directly on the ellipse or very very close, it's on the surface
  // std::numeric_limits<T>::epsilon() is in the order of 1e-7 (single precision) and 1e-16 (double precision)
  if (std::abs(ellipse) < std::numeric_limits<T>::epsilon()) {
    if ((z_q > cylinder.height / 2) || (z_q < -cylinder.height / 2)) {
      //std::cout << "YEAAAH " << std::min(dist_top, dist_bottom)  << "\n";
      return std::min(dist_top, dist_bottom);
    } else {
      return static_cast<T>(0);
    }
  }

  // we inside of the ellipse
  if (ellipse < 0) {
    // if above or below, the distance to top/bottom is the correct one
    if (z_q >= cylinder.height / 2)
      return dist_top;
    if (z_q <= -cylinder.height / 2)
      return dist_bottom;

    const auto [point, dist_shell] = find_minimum_distance_to_ellipse(a, b, point_xy);
    return std::min({dist_shell, dist_bottom, dist_top});
  }

  // we are outside the ellipse
  if (ellipse > 0) {
    // the point is next to the cylinder -> only the ellipse can be the closest distance
    if ((z_q < cylinder.height / 2) && (z_q > -cylinder.height / 2)) {
      const auto [point, dist_shell] = find_minimum_distance_to_ellipse(a, b, point_xy);

      return dist_shell;
    } else {
      //std::cout << "YEEEEAH\n";
      // we are above and next to it ->  distance_to_ellipse^2 + distance_(top/bottom)^2
      const auto [point, dist_shell] = find_minimum_distance_to_ellipse(a, b, point_xy);

      const T dist1 = std::sqrt(dist_shell * dist_shell + dist_top * dist_top);
      const T dist2 = std::sqrt(dist_shell * dist_shell + dist_bottom * dist_bottom);
      return std::min({dist1, dist2});
    }
  }
  return -1;
}

template<typename T>
std::ostream &operator<<(std::ostream &stream, const Cylinder<T> &cylinder) {
  /// Conveniently output a single cylinder as JSON.
  stream << "{\n";

  stream << R"(  "Cylinder": )" << &cylinder << ",\n";
  stream << R"(  "center": [)" << cylinder.center[0] << "," << cylinder.center[1] << "," << cylinder.center[2]
         << "],\n";
  stream << R"(  "semi_axis_a": )" << cylinder.semi_axis_a << ",\n";
  stream << R"(  "semi_axis_b": )" << cylinder.semi_axis_b << ",\n";
  stream << R"(  "height": )" << cylinder.height << ",\n";
  stream << R"(  "rotation": [[)" << cylinder.rotation(0, 0) << ", " << cylinder.rotation(0, 1) << ", "
         << cylinder.rotation(0, 2) << "],\n"
         << "             [" << cylinder.rotation(1, 0) << ", " << cylinder.rotation(1, 1) << ", "
         << cylinder.rotation(1, 2) << "],\n"
         << "             [" << cylinder.rotation(2, 0) << ", " << cylinder.rotation(2, 1) << ", "
         << cylinder.rotation(2, 2) << "]],\n";
  stream << R"(  "prim_id": )" << cylinder.prim_id << "\n";

  stream << "}\n";
  return stream;
}

template<typename T>
std::ostream &operator<<(std::ostream &stream, const CylinderCollection<T> &collection) {
  stream << "{\n";
  stream << R"("CylinderCollection": [)"
         << "\n";
  stream << R"({ "size": )" << collection.size() << "},\n";

  for (uint32_t id_cylinder = 0; id_cylinder < collection.size(); id_cylinder++) {
    stream << primitive_from_collection(collection, id_cylinder);
    if (id_cylinder == collection.size() - 1) {
      stream << "]\n";
    } else {
      stream << ", \n";
    }
  }

  stream << "}\n";
  return stream;
}

}// namespace blazert

#endif//BLAZERT_CYLINDER_H
