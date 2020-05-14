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
  const Vec3rList<T> *centers;
  const std::vector<T> *radii;

public:
  Sphere() = default;
  Sphere(const Vec3rList<T> &centers, const std::vector<T>& radii) : centers(&centers), radii(&radii) {};

  /**
   * @brief Returns the number of primitives in the Sphere -> only one sphere
   * @return unsigned int
   */
  inline unsigned int size() const { return 1; }

  inline void BoundingBox(Vec3r<T> &bmin, Vec3r<T> &bmax, unsigned int prim_index) const {
    bmin = (*centers)[prim_index] - (*radii)[prim_index];
    bmax = (*centers)[prim_index] + (*radii)[prim_index];
  }

  inline void BoundingBoxAndCenter(Vec3r<T> &bmin, Vec3r<T> &bmax, Vec3r<T> &center, unsigned int prim_index) const {
    BoundingBox(bmin, bmax, prim_index);
    center = (*centers)[prim_index];
  }
};

// Predefined SAH predicator for Sphere.
template<typename T>
class SphereSAHPred {
private:
  mutable unsigned int axis_;
  mutable T pos_;
  const Vec3rList<T> *centers;
  const std::vector<T> *radii;

public:
  SphereSAHPred() = default;
  SphereSAHPred(const Vec3rList<T> &centers, const std::vector<T> &radii)
      : axis_(0), pos_(static_cast<T>(0.0)), centers(&centers), radii(&radii) {}

  SphereSAHPred(const SphereSAHPred<T> &rhs)
      : axis_(rhs.axis_),
        pos_(rhs.pos_),
        centers(rhs.centers),
        radii(rhs.radii) {}

  SphereSAHPred<T> &operator=(const SphereSAHPred<T> &rhs) {
    axis_ = rhs.axis_;
    pos_ = rhs.pos_;
    centers = rhs.centers;
    radii = rhs.radii;
    return (*this);
  }

  void Set(unsigned int axis, T pos) const {
    axis_ = axis;
    pos_ = pos;
  }

  /// The operator returns true, if the Sphere center is within a margin of 3 time the position along a specified axis
  inline bool operator()(unsigned int prim_id) const {
    T center = (*centers)[prim_id][axis_];
    return (center < pos_);
  }
};

template<typename T>
class SphereIntersector {
public:
  const Vec3rList<T> *centers;
  const std::vector<T> *radii;

  mutable Vec3r<T> ray_org_;
  mutable Vec3r<T> ray_dir_;
  mutable T t_min_;
  mutable Vec2r<T> uv_;
  mutable T hit_distance;
  mutable unsigned int prim_id_;
  mutable bool cull_back_face;

public:
  inline SphereIntersector(const Vec3rList<T> &centers, const std::vector<T> &radii) : centers(&centers), radii(&radii), prim_id_(-1), cull_back_face(false) {}
};

/// Update is called when initializing intersection and nearest hit is found.
template<typename T>
inline void update_intersector(SphereIntersector<T> &i, const T t, const unsigned int prim_id) {
  i.hit_distance = t;
  i.prim_id_ = prim_id;
}

/**
   * Post BVH traversal stuff.
   * Fill `isect` if there is a hit.
   * TODO: Is Ray<T> really needed here?
   */
template<typename T>
inline void post_traversal(SphereIntersector<T> &i, const Ray<T> &ray, const bool hit, RayHit<T> &intersection) {
  if (hit) {
    intersection.hit_distance = i.hit_distance;
    intersection.uv = i.uv_;
    intersection.prim_id = i.prim_id_;
    intersection.normal = normalize(i.ray_org_ + i.hit_distance * i.ray_dir_ - (*i.centers)[i.prim_id_]);
  }
}

/**
   * Prepare BVH traversal (e.g. compute inverse ray direction).
   * This function is called only once in BVH traversal.
   */
template<typename T>
inline void prepare_traversal(SphereIntersector<T> &i, const Ray<T> &ray, const BVHTraceOptions<T> &trace_options) {

  i.ray_org_ = ray.origin;// copy here we'll have an allocate?
  i.ray_dir_ = ray.direction;

  i.t_min_ = ray.min_hit_distance;

  i.uv_ = {0., 0.};// Here happens an allocate.
  i.cull_back_face = trace_options.cull_back_face;
}

/**
   * Do ray intersection stuff for `prim_index` th primitive and return hit
   * distance `hit_distance`, barycentric coordinate `u` and `v`.
   * Returns true if there's intersection.
   */
template<typename T>
inline bool intersect(SphereIntersector<T> &i, T &t_inout, const unsigned int prim_index) {

  const Vec3r<T> &org = i.ray_org_;
  const Vec3r<T> &dir = i.ray_dir_;

  const Vec3r<T>& center = (*i.centers)[prim_index];
  const T radius = (*i.radii)[prim_index];

  const Vec3r<T> &v = org - center;

  // calculate the actual intersections
  const T A = dot(dir, dir);
  const T B = static_cast<T>(2.0) * dot(v, dir);                       //
  const T C = dot(v, v) - radius * radius;// distance to sphere
  const T D = B * B - static_cast<T>(4.0) * A * C;

  // interpretation of D: if D < 0 the ray misses the sphere and therefore
  // does not intersect term in the square root becomes negative
  if (D < static_cast<T>(0.0)) return false;

  const T Q = sqrtf(D);
  const T rcpA = static_cast<T>(1.) / A;
  const T t0 = static_cast<T>(0.5) * rcpA * (-B - Q);
  const T t1 = static_cast<T>(0.5) * rcpA * (-B + Q);

  // t0 corresponds to the first hit
  // hit distance is set to maximum in update_intersector
  if ((t0 > i.t_min_) && (t0 < i.hit_distance)) {
    t_inout = t0;
    return true;
  }
  // t1 corresponds to the first hit
  if ((t1 > i.t_min_) && (t1 < i.hit_distance)) {
    t_inout = t1;
    return true;
  }
  return false;
}

template<typename T>
T distance_to_surface(Sphere<T> &sphere, const Vec3r<T>& point,  const unsigned int prim_index)
{
  const Vec3r<T>& distance = (*sphere.centers)[prim_index] - point;
  return abs(norm(distance) - (*sphere.radii)[prim_index]);
}

}// namespace blazert

#endif// BLAZERT_PRIMITIVES_SPHERE_H_
