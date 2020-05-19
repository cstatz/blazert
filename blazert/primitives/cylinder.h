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
  const Vec3rList<T> *centers;// center of bottom ellipse
  const std::vector<T> *semi_axes_a;
  const std::vector<T> *semi_axes_b;
  const std::vector<T> *heights;
  const Mat3rList<T> *rotations;

public:
  Cylinder() = default;
  Cylinder(const Vec3rList<T> &centers, const std::vector<T> &semi_axis_a, const std::vector<T> &semi_axis_b, const std::vector<T> &heights, const Mat3rList<T> &rotations)
      : centers(&centers), semi_axes_a(&semi_axis_a), semi_axes_b(&semi_axis_b), heights(&heights), rotations(&rotations){};

  /**
   * @brief Returns the number of primitives in the cylinder -> only one cylinder
   * @return unsigned int
   */
  inline unsigned int size() const { return centers->size(); }

  inline void BoundingBox(Vec3r<T> &bmin, Vec3r<T> &bmax, unsigned int prim_index) const {
    const Vec3r<T> &center = (*centers)[prim_index];
    const T a = (*semi_axes_a)[prim_index];
    const T b = (*semi_axes_b)[prim_index];
    const T height = (*heights)[prim_index];
    const Mat3r<T> &rotation = (*rotations)[prim_index];

    const Vec3r<T> &a1_tmp{a, 0, 0};
    const Vec3r<T> &b1_tmp{0, b, 0};
    const Vec3r<T> &h1_tmp{0, 0, height};
    const Vec3r<T> &a2_tmp{a, 0, 0};
    const Vec3r<T> &b2_tmp{0, b, 0};
    //const Vec3r<T> &h2_tmp{0, 0, height};

    // These vectors describe the cylinder in the global coordinate system
    const Vec3r<T> &a1 = center + rotation * a1_tmp;
    const Vec3r<T> &b1 = center + rotation * b1_tmp;
    const Vec3r<T> &h1 = center + rotation * h1_tmp;

    const Vec3r<T> &a2 = center - rotation * a2_tmp;
    const Vec3r<T> &b2 = center - rotation * b2_tmp;

    // maximum / minimum is also the max/min of the bounding box
    bmin[0] = std::min({ a1[0], b1[0], a2[0], b2[0], h1[0] });
    bmin[1] = std::min({ a1[1], b1[1], a2[1], b2[1], h1[1] });
    bmin[2] = std::min({ a1[2], b1[2], a2[2], b2[2], h1[2] });
    bmax[0] = std::max({ a1[0], b1[0], a2[0], b2[0], h1[0] });
    bmax[1] = std::max({ a1[1], b1[1], a2[1], b2[1], h1[1] });
    bmax[2] = std::max({ a1[2], b1[2], a2[2], b2[2], h1[2] });
  }

  inline void BoundingBoxAndCenter(Vec3r<T> &bmin, Vec3r<T> &bmax, Vec3r<T> &center, unsigned int prim_index) const {
    BoundingBox(bmin, bmax, prim_index);
    center = (*centers)[prim_index] + (*rotations)[prim_index] * Vec3r<T>{0,0,(*heights)[prim_index]/2};
  }
};

// Predefined SAH predicator for cylinder.
template<typename T>
class CylinderSAHPred {
private:
  mutable unsigned int axis_;
  mutable T pos_;
  const Vec3rList<T> *centers;// center of bottom ellipse
  const std::vector<T> *semi_axes_a;
  const std::vector<T> *semi_axes_b;
  const std::vector<T> *heights;
  const Mat3rList<T> *rotations;

public:
  CylinderSAHPred() = default;
  CylinderSAHPred(const Vec3rList<T> &centers, const std::vector<T> &semi_axis_a, const std::vector<T> &semi_axis_b, const std::vector<T> &heights, const Mat3rList<T> &rotations)
      : axis_(0), pos_(static_cast<T>(0.0)), centers(&centers), semi_axes_a(&semi_axis_a), semi_axes_b(&semi_axis_b), heights(&heights), rotations(&rotations) {}

  // TODO: fix constructor according to data members
  CylinderSAHPred(const CylinderSAHPred<T> &rhs)
      : axis_(rhs.axis_),
        pos_(rhs.pos_),
        centers(rhs.centers),
        semi_axes_a(rhs.semi_axes_a),
        semi_axes_b(rhs.semi_axes_b),
        heights(rhs.heights),
        rotations(rhs.rotations) {}
  // TODO: fix constructor according to data members
  CylinderSAHPred<T> &operator=(const CylinderSAHPred<T> &rhs) {
    axis_ = rhs.axis_;
    pos_ = rhs.pos_;
    centers = rhs.centers;
    semi_axes_a = rhs.semi_axes_a;
    semi_axes_b = rhs.semi_axes_b;
    heights = rhs.heights;
    rotations = rhs.rotations;
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
class CylinderIntersector {
public:
  const Vec3rList<T> *centers;// center of bottom ellipse
  const std::vector<T> *semi_axes_a;
  const std::vector<T> *semi_axes_b;
  const std::vector<T> *heights;
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
  inline CylinderIntersector(const Vec3rList<T> &centers, const std::vector<T> &semi_axis_a, const std::vector<T> &semi_axis_b, const std::vector<T> &heights, const Mat3rList<T> &rotations)
      : centers(&centers), semi_axes_a(&semi_axis_a), semi_axes_b(&semi_axis_b), heights(&heights), rotations(&rotations), prim_id_(-1), cull_back_face(false) {}
};

/// Update is called when initializing intersection and nearest hit is found.
template<typename T>
inline void update_intersector(CylinderIntersector<T> &i, const T t, const unsigned int prim_id) {
  i.hit_distance = t;
  i.prim_id_ = prim_id;
}

/**
 * Post BVH traversal stuff.
 * Fill `isect` if there is a hit.
 * TODO: Is Ray<T> really needed here?
 */
template<typename T>
inline void post_traversal(CylinderIntersector<T> &i, const Ray<T> &ray, const bool hit, RayHit<T> &intersection) {
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
inline void prepare_traversal(CylinderIntersector<T> &i, const Ray<T> &ray, const BVHTraceOptions<T> &trace_options) {

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
inline bool intersect(CylinderIntersector<T> &i, T &t_inout, const unsigned int prim_index) {

  const Vec3r<T> &center_ = (*i.centers)[prim_index];
  const T semi_axis_a = (*i.semi_axes_a)[prim_index];
  const T semi_axis_b = (*i.semi_axes_b)[prim_index];
  const T height = (*i.heights)[prim_index];
  const Mat3r<T> &rotation = (*i.rotations)[prim_index];
  const Mat3r<T> &inverse_rotation = trans(rotation);

  // center on coordinate system of object
  const Vec3r<T> &org = inverse_rotation * (i.ray_org_ - center_);
  const Vec3r<T> &dir = inverse_rotation * i.ray_dir_;
  //const Vec3r<T> &center = inverse_rotation * center_;

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
  if (z0 > h) {
    if (l == 0)
      return false;
    const T t0 = (h - z0) / l;
    if (t0 < 0)
      return false;

    const Vec3r<T> intercept = org + /*center + */ t0 * dir;

    // intercept point in circle
    if (b * b * intercept[0] * intercept[0] + a * a * intercept[1] * intercept[1] <= a * a * b * b) {
      const Vec3r<T> normal{0.f, 0.f, 1.f};
      const Vec3r<T> Ng = rotation * normal / norm(normal);
      i.normal = Ng;
      t_inout = norm(t0*dir);
      return true;
    } else {
      const T A = a * a * k * k + b * b * j * j;
      const T B = a * a * k * y0 + b * b * j * x0;
      const T C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return false;
      
      const T t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const T t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<T> intercept0 = org + /*center + */ dir * t0;
      const Vec3r<T> intercept1 = org + /*center + */ dir * t1;

      // t1 is right intercept
      if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h) && (intercept1[2] >= 0)) {
        const Vec3r<T> normal{2 *  intercept1[0] / (a * a), 2 *  intercept1[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        t_inout = norm(t1*dir);
        return true;
      }
      if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h) && (intercept0[2] >= 0)) {
        const Vec3r<T> normal{2 *  intercept0[0] / (a * a), 2 *  intercept0[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        t_inout = norm(t0*dir);
        return true;
      }
    }
  } else
    // area 2
  if (z0 < 0) {
    if (l == 0)
      return false;
    const T t0 = -z0 / l;
    if (t0 < 0)
      return false;

    const Vec3r<T> intercept = org + /*center + */ t0 * dir;

    // intercept point in circle
    if (b * b * intercept[0] * intercept[0] + a * a * intercept[1] * intercept[1] <= a * a * b * b) {
      const Vec3r<T> normal{0.f, 0.f, -1.f};
      const Vec3r<T> Ng = rotation * normal / norm(normal);
      i.normal = Ng;
      t_inout = norm(t0*dir);
      return true;
    } else {
      const T A = a * a * k * k + b * b * j * j;
      const T B = a * a * k * y0 + b * b * j * x0;
      const T C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return false;

      const T t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const T t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<T> intercept0 = org + /*center + */ dir * t0;
      const Vec3r<T> intercept1 = org + /*center + */ dir * t1;
      // t1 is right intercept
      if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h) && (intercept1[2] >= 0)) {
        const Vec3r<T> normal{2 *  intercept1[0] / (a * a), 2 *  intercept1[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        t_inout = norm(t1*dir);
        return true;
      }
      if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h) && (intercept0[2] >= 0)) {
        const Vec3r<T> normal{2 *  intercept0[0] / (a * a), 2 *  intercept0[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        t_inout = norm(t0*dir);
        return true;
      }
    }
  }
    // area 3
  else if ((z0 > 0) && (z0 < h) && (x0 * x0 * b * b + y0 * y0 * a * a > a * a * b * b)) {
    const T A = a * a * k * k + b * b * j * j;
    const T B = a * a * k * y0 + b * b * j * x0;
    const T C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

    if (C < 0)
      return false;
    const T t0 = 1.0 / A * (-B + a * b * sqrt(C));
    const T t1 = -1.0 / A * (B + a * b * sqrt(C));

    const Vec3r<T> intercept0 = org + /*center + */ dir * t0;
    const Vec3r<T> intercept1 = org + /*center + */ dir * t1;
    // t1 is right intercept
    if ((t1 > 0) && (t1 < t0) && (intercept1[2] <= h) && (intercept1[2] >= 0)) {
      const Vec3r<T> normal{2 *  intercept1[0] / (a * a), 2 *  intercept1[1] / (b * b), 0.f};
      const Vec3r<T> Ng = rotation * normal / norm(normal);
      i.normal = Ng;
      t_inout = norm(t1*dir);
      return true;
    }
    if ((t0 > 0) && (t0 < t1) && (intercept0[2] <= h) && (intercept0[2] >= 0)) {
      const Vec3r<T> normal{2 *  intercept0[0] / (a * a), 2 *  intercept0[1] / (b * b), 0.f};
      const Vec3r<T> Ng = rotation * normal / norm(normal);
      i.normal = Ng;
      t_inout = norm(t0*dir);
      return true;
    }
  }
  // area 4
  if ((z0 >= 0) && (z0 <= h) && (x0 * x0 * b * b + y0 * y0 * a * a <= a * a * b * b)) {
    // only surrounding surface can have an intersection
    if (l == 0) {
      // if the z-component of the direction vector points along the
      // bottom or top circle, we will not have a intersection
      if ((z0 == 0) || (z0 == h))
        return false;

      const T A = a * a * k * k + b * b * j * j;
      const T B = a * a * k * y0 + b * b * j * x0;
      const T C = a * a * k * k + b * b * j * j - j * j * y0 * y0 + 2 * j * k * x0 * y0 - k * k * x0 * x0;

      if (C < 0)
        return false;

      const T t0 = 1.0 / A * (-B + a * b * sqrt(C));
      const T t1 = -1.0 / A * (B + a * b * sqrt(C));

      const Vec3r<T> intercept0 = org + /*center + */ dir * t0;
      const Vec3r<T> intercept1 = org + /*center + */ dir * t1;
      // t1 is right intercept
      if ((t1 > 0) && (intercept1[2] <= h) && (intercept1[2] >= 0)) {
        const Vec3r<T> normal{2 *  intercept1[0] / (a * a), 2 *  intercept1[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        t_inout = norm(t1*dir);
        return true;
      }
      if ((t0 > 0) && (intercept0[2] <= h) && (intercept0[2] >= 0)) {
        const Vec3r<T> normal{2 *  intercept0[0] / (a * a), 2 *  intercept0[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        t_inout = norm(t0*dir);
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
        const T t0 = -z0 / l;

        const Vec3r<T> normal{0.f, 0.f, -1.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        t_inout = norm(t0*dir);
        return true;
      }
      // top circle
      if (l > 0) {
        const T t0 = (h - z0) / l;

        const Vec3r<T> normal{0.f, 0.f, 1.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        t_inout = norm(t0*dir);
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

      const Vec3r<T> intercept0 = org + /*center + */ dir * t0;
      const Vec3r<T> intercept1 = org + /*center + */ dir * t1;
      // t1 is right intercept

      if ((t1 > 0) && (intercept1[2] <= h) && (intercept1[2] >= 0)) {
        const Vec3r<T> normal{2 *  intercept1[0] / (a * a), 2 *  intercept1[1] / (b * b), 0.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        t_inout = norm(t1*dir);
        return true;
      }
      if ((t0 > 0) && (intercept0[2] <= h) && (intercept0[2] >= 0)) {
        const Vec3r<T> normal{2 *  intercept0[0] / (a * a), 2 *  intercept0[1] / (b * b), 0.f};
        i.normal = normal;
        t_inout = norm(t0*dir);
        return true;
      }

      // bottom circle
      if (l < 0) {
        const T t0 = -z0 / l;

        const Vec3r<T> normal{0.f, 0.f, -1.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        t_inout = norm(t0*dir);
        return true;
      }
      // top circle
      if (l > 0) {
        const T t0 = (h - z0) / l;

        const Vec3r<T> normal{0.f, 0.f, 1.f};
        const Vec3r<T> Ng = rotation * normal / norm(normal);
        i.normal = Ng;
        t_inout = norm(t0*dir);
        return true;
      }
    }
  }

  return false;
}

template<typename T>
double distance_to_surface(Cylinder<T> &cylinder, const Vec3r<T> &point, const unsigned int prim_index) {
  /**
   * TODO: calculate the distance to the surface of the object from point
   */
  /***
   * TODO: calculate distance to top and bottom, and shell surface, take minimum
   */
  return 0;
}

}// namespace blazert

#endif//BLAZERT_CYLINDER_H
