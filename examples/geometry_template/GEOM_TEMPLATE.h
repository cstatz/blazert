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
class GEOM_NAME {

public:
  /**
   * TODO: Add properties of the object, e.g. centers, radius, rotation matrices, ...
   *
   * examples for sphere (needs center and a radius):
   * const Vec3rList<T> *centers;
   * const std::vector<T> *radii;
   *
   * for blaze types, use Vec3rList<T> to make use of alligned allocator
   * otherwise use std::vector<T>
   * T will be double or float
   */

public:
  GEOM_NAME() = default;
  GEOM_NAME(/*TODO: const Vec3rList<T> &centers, const std::vector<T>& radii*/) : centers(&centers), radii(&radii){};

  /**
   * @brief Returns the number of primitives in the GEOM_NAME -> only one GEOM_NAME
   * @return unsigned int
   */
  inline unsigned int size() const { return 1; }

  inline void BoundingBox(Vec3r<T> &bmin, Vec3r<T> &bmax, unsigned int prim_index) const {
    /**
     * TODO: calculate bounding box
     */
  }

  inline void BoundingBoxAndCenter(Vec3r<T> &bmin, Vec3r<T> &bmax, Vec3r<T> &center, unsigned int prim_index) const {
    /** 
     * TODO: call BoundigBox and calculate center 
     */
  }
};

// Predefined SAH predicator for GEOM_NAME.
template<typename T>
class GEOM_NAMESAHPred {
private:
  mutable unsigned int axis_;
  mutable T pos_;
  /**
   * TODO: Add properties of the object, e.g. centers, radius, rotation matrices, ... see GEOM_NAME
   */

public:
  GEOM_NAMESAHPred() = default;
  GEOM_NAMESAHPred(/*TODO: const Vec3rList<T> &centers, const std::vector<T> &radii*/)
      : axis_(0), pos_(static_cast<T>(0.0)), centers(&centers), radii(&radii) {}

  // TODO: fix constructor according to data members
  GEOM_NAMESAHPred(const GEOM_NAMESAHPred<T> &rhs)
      : axis_(rhs.axis_),
        pos_(rhs.pos_),
        centers(rhs.centers),
        radii(rhs.radii) {}
  // TODO: fix constructor according to data members
  GEOM_NAMESAHPred<T> &operator=(const GEOM_NAMESAHPred<T> &rhs) {
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

  inline bool operator()(unsigned int prim_id) const {
    /*
     * TODO: calculate SAHPredictor, for simple primitives, center will be good enough
     */
    T center = (*centers)[prim_id][axis_];
    return (center < pos_);
  }
};

template<typename T>
class GEOM_NAMEIntersector {
public:
  /**
   * TODO: Add properties of the object, e.g. centers, radius, rotation matrices, ... see GEOM_NAME
   */

  mutable Vec3r<T> ray_org_;
  mutable Vec3r<T> ray_dir_;
  mutable T t_min_;
  mutable Vec2r<T> uv_;
  mutable T hit_distance;
  mutable unsigned int prim_id_;
  mutable bool cull_back_face;

public:
  inline GEOM_NAMEIntersector(/*TODO: const Vec3rList<T> &centers, const std::vector<T> &radii*/) : centers(&centers), radii(&radii), prim_id_(-1), cull_back_face(false) {}
};

/// Update is called when initializing intersection and nearest hit is found.
template<typename T>
inline void update_intersector(GEOM_NAMEIntersector<T> &i, const T t, const unsigned int prim_id) {
  i.hit_distance = t;
  i.prim_id_ = prim_id;
}

/**
 * Post BVH traversal stuff.
 * Fill `isect` if there is a hit.
 * TODO: Is Ray<T> really needed here?
 */
template<typename T>
inline void post_traversal(GEOM_NAMEIntersector<T> &i, const Ray<T> &ray, const bool hit, RayHit<T> &intersection) {
  if (hit) {
    intersection.hit_distance = i.hit_distance;
    intersection.uv = i.uv_;
    intersection.prim_id = i.prim_id_;
    // TODO: Calculate normal vector of geometry
    // e.g. sphere: intersection.normal = normalize(i.ray_org_ + i.hit_distance * i.ray_dir_ - (*i.centers)[i.prim_id_]);
  }
}

/**
   * Prepare BVH traversal (e.g. compute inverse ray direction).
   * This function is called only once in BVH traversal.
   */
template<typename T>
inline void prepare_traversal(GEOM_NAMEIntersector<T> &i, const Ray<T> &ray, const BVHTraceOptions<T> &trace_options) {

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
inline bool intersect(GEOM_NAMEIntersector<T> &i, T &t_inout, const unsigned int prim_index) {

  const Vec3r<T> &org = i.ray_org_;
  const Vec3r<T> &dir = i.ray_dir_;

  /** TODO: Load all necessary information from intersector and calculate intersections
   * -> save the ray distance in t_inout
   * -> return true if hit, false otherwise
   */

  return false;
}

template<typename T>
double distance_to_surface(GEOM_NAME<T> &GEOM_NAME, const Vec3r<T> &point, const unsigned int prim_index) {
  /**
   * TODO: calculate the distance to the surface of the object from point
   */
  const Vec3r<T> &distance = (*GEOM_NAME.centers)[prim_index] - point;
  return abs(norm(distance) - (*GEOM_NAME.radii)[prim_index]);
}

}// namespace blazert

#endif//BLAZERT_GEOM_TEMPLATE_H
