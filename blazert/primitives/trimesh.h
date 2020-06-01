#pragma once
#ifndef BLAZERT_PRIMITIVES_TRIMESH_H_
#define BLAZERT_PRIMITIVES_TRIMESH_H_

#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

#include <blazert/bvh/options.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/trimesh_distance.h>
#include <blazert/ray.h>

namespace blazert {

template<typename T>
class TriangleMesh {

public:
  const Vec3rList<T> &vertices;
  const Vec3iList &faces;

public:
  TriangleMesh() = delete;
  TriangleMesh(const Vec3rList<T> &vertices, const Vec3iList &faces) : vertices(vertices), faces(faces) {
    // Compute centers
    // Compute face normals
    // Compute vertex normals
  }

  /**
   * @brief Returns the number of primitives in the triangle mesh
   * @return unsigned int
   */
  [[nodiscard]] inline unsigned int size() const { return faces.size(); }

  inline void BoundingBox(Vec3r<T> &bmin, Vec3r<T> &bmax, unsigned int prim_index) const {

    const Vec3ui &face = faces[prim_index];

    {
      const Vec3r<T> &vertex = vertices[face[0]];
      bmin = bmax = vertex;
    }

    for (unsigned int i = 1; i < 3; i++) {
      const Vec3r<T> &vertex = vertices[face[i]];
      for (int k = 0; k < 3; k++) {
        bmin[k] = std::min(bmin[k], vertex[k]);
        bmax[k] = std::max(bmax[k], vertex[k]);
      }
    }
  }

  inline void BoundingBoxAndCenter(Vec3r<T> &bmin, Vec3r<T> &bmax, Vec3r<T> &center, unsigned int prim_index) const {
    BoundingBox(bmin, bmax, prim_index);
    const Vec3ui &face = faces[prim_index];
    center = (vertices[face[0]] + vertices[face[1]] + vertices[face[2]]) / static_cast<T>(3.);
  }
};

// Predefined SAH predicator for triangle.
template<typename T>
class TriangleSAHPred {
private:
  mutable unsigned int axis_;
  mutable T pos_;
  const Vec3rList<T> &vertices;
  const Vec3iList &faces;

public:
  TriangleSAHPred() = default;
  TriangleSAHPred(const Vec3rList<T> &vertices, const Vec3iList &faces)
      : axis_(0), pos_(static_cast<T>(0.0)), vertices(vertices), faces(faces) {}

  void Set(unsigned int axis, T pos) const {
    axis_ = axis;
    pos_ = pos;
  }

  /// The operator returns true, if the triangle center is within a margin of 3 time the position along a specified axis
  // TODO: This SAH predictor is terrible if the triangles are small w.r.t. the scene.
  inline bool operator()(unsigned int prim_id) const {

    // use precomputed center.
    // compute circumference from center and point (+margin), use this as a marker for the sah prediction
    const Vec3ui &face = faces[prim_id];
    const Vec3r<T> &p0 = vertices[face[0]];
    const Vec3r<T> &p1 = vertices[face[1]];
    const Vec3r<T> &p2 = vertices[face[2]];

    const T center = p0[axis_] + p1[axis_] + p2[axis_];

    return (center < pos_ * static_cast<T>(3.0));
  }
};

template<typename T>
class TriangleIntersector {
public:
  const Vec3rList<T> &vertices;
  const Vec3iList &faces;

  mutable Vec3r<T> origin;
  mutable T min_hit_distance;
  mutable Vec2r<T> uv;
  mutable T hit_distance;
  mutable unsigned int prim_id;

public:
  inline TriangleIntersector(const Vec3rList<T> &vertices, const Vec3iList &faces)
      : vertices(vertices), faces(faces), prim_id(-1) {}
};

/// Update is called when initializing intersection and nearest hit is found.

/**
   * Post BVH traversal stuff.
   * Fill `isect` if there is a hit.
   * TODO: Is Ray<T> really needed here?
   */
template<typename T>
inline void post_traversal(TriangleIntersector<T> &i, RayHit<T> &intersection) {
    intersection.hit_distance = i.hit_distance;
    intersection.uv = i.uv;
    intersection.prim_id = i.prim_id;
    // TODO: Only do the normal computation if the prim is hit!
    //intersection.normal = normal_;
}

/**
   * Prepare BVH traversal (e.g. compute inverse ray direction).
   * This function is called only once in BVH traversal.
   */
template<typename T>
inline void prepare_traversal(TriangleIntersector<T> &i, const Ray<T> &ray) {

  // Copy
  //i.origin = ray.origin;
  i.min_hit_distance = ray.min_hit_distance;
  i.hit_distance = ray.max_hit_distance;
  i.uv = 0.; //{0., 0.};  // Here happens an allocate.
  //i.cull_back_face = ray.cull_back_face;  //trace_options.cull_back_face;
  i.prim_id = -1;
  /**
  const Vec3r<T> &dir = ray.direction;

  // Calculate dimension where the ray direction is maximal.
  // TODO: Vectorize this.
  i.k[2] = 0;
  T abs_dir = std::abs(dir[0]);

  if (abs_dir < std::abs(dir[1])) {
    i.k[2] = 1;
    abs_dir = std::abs(dir[1]);
  }

  if (abs_dir < std::abs(dir[2]))
    i.k[2] = 2;

  i.k[0] = i.k[2] + 1;
  if (i.k[0] == 3)
    i.k[0] = 0;

  i.k[1] = i.k[0] + 1;
  if (i.k[1] == 3)
    i.k[1] = 0;

  // Swap kx and ky dimension to preserve winding direction of triangles.
  if (dir[i.k[2]] < static_cast<T>(0.0)) std::swap(i.k[1], i.k[2]);

  // TODO: Removed static_cast. Was it really necessary here?
  i.s[0] = -dir[i.k[0]] / dir[i.k[2]];
  i.s[1] = -dir[i.k[1]] / dir[i.k[2]];
  i.s[2] = static_cast<T>(1.) / dir[i.k[2]];
  */
}

/// Safe function to reinterpret the bits of the given value as another type.
template <typename To, typename From>
To as(From from) {
  static_assert(sizeof(To) == sizeof(From));
  To to;
  std::memcpy(&to, &from, sizeof(from));
  return to;
}

/// Equivalent to copysign(x, x * y).
inline float product_sign(float x, float y) {
  return as<float>(as<uint32_t>(x) ^ (as<uint32_t>(y) & UINT32_C(0x80000000)));
}

/// Equivalent to copysign(x, x * y).
inline double product_sign(double x, double y) {
  return as<double>(as<uint64_t>(x) ^ (as<uint64_t>(y) & UINT64_C(0x8000000000000000)));
}

template<typename T>
inline bool intersect2(TriangleIntersector<T> &i, const Tri<T> &tri, const Ray<T> &ray) {
  static constexpr T tolerance = 4*std::numeric_limits<T>::epsilon();//T(1e-12);

  const auto &e2 = tri.b; //tri.c - tri.a;
  const auto &e1 = tri.c; //tri.a - tri.b;

  const auto c = tri.a - ray.origin;
  const auto r = cross(ray.direction, c);
  const auto det = dot(cross(e1, e2), ray.direction);
  const auto abs_det = std::abs(det);

  const auto u = product_sign(dot(r, e2), det);
  const auto v = product_sign(dot(r, e1), det);
  const auto w = abs_det - u - v;

  if (u >= -tolerance && v >= -tolerance && w >= -tolerance) {
    const auto t = product_sign(dot(cross(e1, e2), c), det);
    if (t >= abs_det * i.min_hit_distance && abs_det * i.hit_distance > t) {
      const auto inv_det = static_cast<T>(1.0) / abs_det;
      i.hit_distance = t*inv_det;
      i.uv = {u*inv_det, v*inv_det};
      i.prim_id = tri.i;
      return true;
    }
  }
  return false;
}

/**
   * Do ray intersection stuff for `prim_index` th primitive and return hit
   * distance `hit_distance`, barycentric coordinate `u` and `v`.
   * Returns true if there's intersection.
   */
template<typename T>
inline bool intersect(TriangleIntersector<T> &i, const Tri<T> &tri, const Ray<T> &ray) {

  const Vec3ui &k = i.k;
  const Vec3r<T> &s = i.s;
  const Vec3r<T> A{tri.a - ray.origin};
  const Vec3r<T> B{tri.b - ray.origin};
  const Vec3r<T> C{tri.c - ray.origin};

  const T Ax = A[k[0]] + s[0] * A[k[2]];
  const T Ay = A[k[1]] + s[1] * A[k[2]];
  const T Bx = B[k[0]] + s[0] * B[k[2]];
  const T By = B[k[1]] + s[1] * B[k[2]];
  const T Cx = C[k[0]] + s[0] * C[k[2]];
  const T Cy = C[k[1]] + s[1] * C[k[2]];

  T U = Cx * By - Cy * Bx;
  T V = Ax * Cy - Ay * Cx;
  T W = Bx * Ay - By * Ax;

  /**
  // Fall back to test against edges using double precision.
  if constexpr (std::is_same<T, float>::value) {
    if (U == static_cast<T>(0.) || V == static_cast<T>(0.) || W == static_cast<T>(0.)) {
      double CxBy = static_cast<double>(Cx) * static_cast<double>(By);
      double CyBx = static_cast<double>(Cy) * static_cast<double>(Bx);
      U = static_cast<T>(CxBy - CyBx);

      double AxCy = static_cast<double>(Ax) * static_cast<double>(Cy);
      double AyCx = static_cast<double>(Ay) * static_cast<double>(Cx);
      V = static_cast<T>(AxCy - AyCx);

      double BxAy = static_cast<double>(Bx) * static_cast<double>(Ay);
      double ByAx = static_cast<double>(By) * static_cast<double>(Ax);
      W = static_cast<T>(BxAy - ByAx);
    }
  }
   */

  if (((U < static_cast<T>(0.)) | (V < static_cast<T>(0.)) | (W < static_cast<T>(0.))) && (i.cull_back_face | (U > static_cast<T>(0.)) | (V > static_cast<T>(0.)) | (W > static_cast<T>(0.)))) {
      return false;
  }

  const T det = U + V + W;

  if (det == static_cast<T>(0.0)) return false;

  const T Az = s[2] * A[k[2]];
  const T Bz = s[2] * B[k[2]];
  const T Cz = s[2] * C[k[2]];

  const T D = U * Az + V * Bz + W * Cz;

  const T rcpDet = static_cast<T>(1.0) / det;
  const T tt = D * rcpDet;

  if ((tt >= i.hit_distance) | (tt < i.min_hit_distance)) return false;

  //t_inout = tt;
  /**
     * TODO: Citation needed.
     * Use Möller-Trumbore style barycentric coordinates
     * U + V + W = 1.0 and interp(p) = U * p0 + V * p1 + W * p2
     * We want interp(p) = (1 - u - v) * p0 + u * v1 + v * p2;
     * => u = V, v = W.
     */
  i.uv = {V * rcpDet, W * rcpDet};
  i.hit_distance = tt;
  i.prim_id = tri.i;

  return true;
}

}// namespace blazert

//void Mesh_normalize( Mesh *myself )
//{
//  precompute face normals, face centers!
//  Vert     *vert = myself->vert;
//  Triangle *face = myself->face;
//
//  for( int i=0; i<myself->mNumVerts; i++ ) vert[i].normal = vec3(0.0f);
//
//  for( int i=0; i<myself->mNumFaces; i++ )
//  {
//    const int ia = face[i].v[0];
//    const int ib = face[i].v[1];
//    const int ic = face[i].v[2];
//
//    const vec3 e1 = vert[ia].pos - vert[ib].pos;
//    const vec3 e2 = vert[ic].pos - vert[ib].pos;
//    const vec3 no = cross( e1, e2 );
//
//    vert[ia].normal += no;
//    vert[ib].normal += no;
//    vert[ic].normal += no;
//  }
//
//  for( i=0; i<myself->mNumVerts; i++ ) verts[i].normal = normalize( verts[i].normal );

//

//}

// Barycentric interpolation: a =  u * a0 + v * a1 + (1-u-v) * a2;

#endif// BLAZERT_PRIMITIVES_TRIMESH_H_
