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
  const Vec3rList<T> *centers;
  const std::vector<T> *dxs;
  const std::vector<T> *dys;
  const Mat3rList<T> *rotations;

  const T thickness = std::numeric_limits<T>::min();

public:
  Plane() = default;
  Plane(const Vec3rList<T> &centers, const std::vector<T> &dxs, const std::vector<T> &dys, const Mat3rList<T> &rotations)
      : centers(&centers), dxs(&dxs), dys(&dys), rotations(&rotations){};

  /**
   * @brief Returns the number of primitives in the Plane -> only one sphere
   * @return unsigned int
   */
  inline unsigned int size() const { return 1; }

  inline void BoundingBox(Vec3r<T> &bmin, Vec3r<T> &bmax, unsigned int prim_index) const {

    const Vec3r<T> &center = (*centers)[prim_index];
    const T dx = (*dxs)[prim_index];
    const T dy = (*dys)[prim_index];
    const Mat3r<T> &rotation = (*rotations)[prim_index];

    // vectors describing extent in x direction
    const Vec3r<T> &a1_tmp{-dx / static_cast<T>(2.0), -dy / static_cast<T>(2.0), 0.0};
    const Vec3r<T> &a2_tmp{dx / static_cast<T>(2.0), -dy / static_cast<T>(2.0), 0.0};
    // vectors describing extent in y direction
    const Vec3r<T> &a3_tmp{-dx / static_cast<T>(2.0), dy / static_cast<T>(2.0), 0.0};
    const Vec3r<T> &a4_tmp{dx / static_cast<T>(2.0), dy / static_cast<T>(2.0), 0.0};

    // vectors describing extent in y direction
    const Vec3r<T> &c1_tmp{0.0, 0.0, -thickness};
    const Vec3r<T> &c2_tmp{0.0, 0.0, thickness};

    // std::cout << rot_internal << std::endl;
    // vectors describing the plane in the global coordinate system
    const Vec3r<T> &a1 = center + rotation * a1_tmp;
    const Vec3r<T> &a2 = center + rotation * a2_tmp;
    const Vec3r<T> &a3 = center + rotation * a3_tmp;
    const Vec3r<T> &a4 = center + rotation * a4_tmp;
    const Vec3r<T> &c1 = center + rotation * c1_tmp;
    const Vec3r<T> &c2 = center + rotation * c2_tmp;

    // maximum / minimum is also the max/min of the bounding box

    bmin[0] = std::min({a1[0], a2[0], a3[0], a4[0], c1[0], c2[0]});
    bmin[1] = std::min({a1[1], a2[1], a3[1], a4[1], c1[1], c2[1]});
    bmin[2] = std::min({a1[2], a2[2], a3[2], a4[2], c1[2], c2[2]});
    bmax[0] = std::max({a1[0], a2[0], a3[0], a4[0], c1[0], c2[0]});
    bmax[1] = std::max({a1[1], a2[1], a3[1], a4[1], c1[1], c2[1]});
    bmax[2] = std::max({a1[2], a2[2], a3[2], a4[2], c1[2], c2[2]});
  } 

  inline void BoundingBoxAndCenter(Vec3r<T> &bmin, Vec3r<T> &bmax, Vec3r<T> &center, unsigned int prim_index) const {
    BoundingBox(bmin, bmax, prim_index);
    center = (*centers)[prim_index];
  }
};

// Predefined SAH predicator for Plane.
template<typename T>
class PlaneSAHPred {
private:
  mutable unsigned int axis_;
  mutable T pos_;
  const Vec3rList<T> *centers;
  const std::vector<T> *dxs;
  const std::vector<T> *dys;
  const Mat3rList<T> *rotations;

public:
  PlaneSAHPred() = default;
  PlaneSAHPred(const Vec3rList<T> &centers, const std::vector<T> &dxs, const std::vector<T> &dys, const Mat3rList<T> &rotations)
      : axis_(0), pos_(static_cast<T>(0.0)), centers(&centers), dxs(&dxs), dys(&dys), rotations(&rotations) {}

  PlaneSAHPred(const PlaneSAHPred<T> &rhs)
      : axis_(rhs.axis_),
        pos_(rhs.pos_),
        centers(rhs.centers),
        dxs(rhs.dxs),
        dys(rhs.dys),
        rotations(rhs.rotations) {}

  PlaneSAHPred<T> &operator=(const PlaneSAHPred<T> &rhs) {
    axis_ = rhs.axis_;
    pos_ = rhs.pos_;
    centers = rhs.centers;
    dxs = rhs.dxs;
    dys = rhs.dys;
    rotations = rhs.rotations;
    return (*this);
  }

  void
  Set(unsigned int axis, T pos) const {
    axis_ = axis;
    pos_ = pos;
  }

  /// The operator returns true, if the Plane center is within a margin of 3 time the position along a specified axis
  inline bool operator()(unsigned int prim_id) const {
    T center = (*centers)[prim_id][axis_];
    return (center < pos_);
  }
};// namespace blazert

template<typename T>
class PlaneIntersector {
public:
  const Vec3rList<T> *centers;
  const std::vector<T> *dxs;
  const std::vector<T> *dys;
  const Mat3rList<T> *rotations;

  // TODO: this might change..
  mutable Vec3r<T> normal;

  mutable Vec3r<T> ray_org_;
  mutable Vec3r<T> ray_dir_;
  mutable T t_min_;
  mutable Vec2r<T> uv_;
  mutable T hit_distance;
  mutable unsigned int prim_id_;
  mutable bool cull_back_face;

public:
  inline PlaneIntersector(const Vec3rList<T> &centers, const std::vector<T> &dxs, const std::vector<T> &dys, const Mat3rList<T> &rotations)
      : centers(&centers), dxs(&dxs), dys(&dys), rotations(&rotations), prim_id_(-1), cull_back_face(false) {}
};

/// Update is called when initializing intersection and nearest hit is found.
template<typename T>
inline void update_intersector(PlaneIntersector<T> &i, const T t, const unsigned int prim_id) {
  i.hit_distance = t;
  i.prim_id_ = prim_id;
}

/**
   * Post BVH traversal stuff.
   * Fill `isect` if there is a hit.
   * TODO: Is Ray<T> really needed here?
   */
template<typename T>
inline void post_traversal(PlaneIntersector<T> &i, const Ray<T> &ray, const bool hit, RayHit<T> &intersection) {
  if (hit) {
    intersection.hit_distance = i.hit_distance;
    intersection.uv = i.uv_;
    intersection.prim_id = i.prim_id_;
    intersection.normal = normalize(i.normal);
  }
}

/**
   * Prepare BVH traversal (e.g. compute inverse ray direction).
   * This function is called only once in BVH traversal.
   */
template<typename T>
inline void prepare_traversal(PlaneIntersector<T> &i, const Ray<T> &ray, const BVHTraceOptions<T> &trace_options) {

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
inline bool intersect(PlaneIntersector<T> &i, T &t_inout, const unsigned int prim_index) {
  // load correct plane from data ptr


  const Vec3r<T> &center = (*i.centers)[prim_index];
  const T dx = (*i.dxs)[prim_index];
  const T dy = (*i.dys)[prim_index];
  const Mat3r<T> &rotation = (*i.rotations)[prim_index];
  const Mat3r<T> &inverse_rotation = trans(rotation);

//  std::cout << "rot = " << rotation << "\n";
//  std::cout << "rot^T = " << inverse_rotation << "\n";

  const Vec3r<T> &org = inverse_rotation * (i.ray_org_ - center);
  const Vec3r<T> &dir = inverse_rotation * i.ray_dir_;

  // calculate interception point
  const T t1 = -org[2] / dir[2];
  const Vec3r<T> &intercept = org + t1 * dir;

  const T tnear = i.t_min_;
  const T tfar = i.hit_distance;

  const T x_min = -dx / 2;
  const T x_max = dx / 2;

  const T y_min = -dy / 2;
  const T y_max = dy / 2;

  // is intercept within ray limits?
  // does it actually hit the plane?
  if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] < x_max) && (intercept[1] > y_min) && (intercept[1] < y_max)) {
    i.normal = org[2] / abs(org[2]) * (rotation * Vec3r<T>{0.0, 0.0, 1.0});
    t_inout = t1;
    return true;
  }

  // plane edges
  else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] == x_min) && (intercept[0] < x_max) && (intercept[1] > y_min) && (intercept[1] < y_max)) {
    i.normal = rotation * Vec3r<T>{-1.0, 0.0, org[2] / abs(org[2])};
    t_inout = t1;
    return true;
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] == x_max) && (intercept[1] > y_min) && (intercept[1] < y_max)) {
    i.normal = rotation * Vec3r<T>{1.0, 0.0, org[2] / abs(org[2])};
    t_inout = t1;
    return true;
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] < x_max) && (intercept[1] == y_min) && (intercept[1] < y_max)) {
    i.normal = rotation * Vec3r<T>{0.0, -1.0, org[2] / abs(org[2])};
    t_inout = t1;
    return true;
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] < x_max) && (intercept[1] > y_min) && (intercept[1] == y_max)) {
    i.normal = rotation * Vec3r<T>{0.0, 1.0, org[2] / abs(org[2])};
    t_inout = t1;
    return true;
  }

  // plane corners
  else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] == x_min) && (intercept[0] < x_max) && (intercept[1] == y_min) && (intercept[1] < y_max)) {
    i.normal = rotation * Vec3r<T>{-1.0, -1.0, org[2] / abs(org[2])};
    t_inout = t1;
    return true;
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] == x_min) && (intercept[0] < x_max) && (intercept[1] > y_min) && (intercept[1] == y_max)) {
    i.normal = rotation * Vec3r<T>{-1.0, 1.0, org[2] / abs(org[2])};
    t_inout = t1;
    return true;
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] == x_max) && (intercept[1] == y_min) && (intercept[1] < y_max)) {
    i.normal = rotation * Vec3r<T>{1.0, -1.0, org[2] / abs(org[2])};
    t_inout = t1;
    return true;
  } else if ((t1 > tnear) && (t1 < tfar) && (intercept[0] > x_min) && (intercept[0] == x_max) && (intercept[1] > y_min) && (intercept[1] == y_max)) {
    i.normal = rotation * Vec3r<T>{1.0, 1.0, org[2] / abs(org[2])};
    t_inout = t1;
    return true;
  }
  return false;
}

template<typename T>
T distance_to_surface(Plane<T> &sphere, const Vec3r<T> &point, const unsigned int prim_index) {
  //TODO
  return 0;
}

}// namespace blazert

#endif//BLAZERT_PLANE_H
