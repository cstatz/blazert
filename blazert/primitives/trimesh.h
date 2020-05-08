#pragma once
#ifndef BLAZERT_PRIMITIVES_TRIMESH_H_
#define BLAZERT_PRIMITIVES_TRIMESH_H_

#include <cmath>
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
  const Vec3rList<T> *vertices_;
  const Vec3iList *faces_;
  const Vec3rList<T> *center_f_;
  const Vec3rList<T> *normals_f_;
  const Vec3rList<T> *normals_v_;

public:
  TriangleMesh() = default;
  TriangleMesh(const Vec3rList<T> &vertices, const Vec3iList &faces) : vertices_(&vertices), faces_(&faces) {
    // Compute centers
    // Compute face normals
    // Compute vertex normals
  }

  /**
   * @brief Returns the number of primitives in the triangle mesh
   * @return unsigned int
   */
  inline unsigned int size() const { return (*faces_).size(); }

  inline void BoundingBox(Vec3r<T> &bmin, Vec3r<T> &bmax, unsigned int prim_index) const {

    const Vec3ui &face = (*faces_)[prim_index];

    {
      const Vec3r<T> &vertex = (*vertices_)[face[0]];
      bmin = bmax = vertex;
    }

    for (unsigned int i = 1; i < 3; i++) {
      const Vec3r<T> vertex = (*vertices_)[face[i]];
      for (int k = 0; k < 3; k++) {
        bmin[k] = std::min(bmin[k], vertex[k]);
        bmax[k] = std::max(bmax[k], vertex[k]);
      }
    }
  }

  inline void BoundingBoxAndCenter(Vec3r<T> &bmin, Vec3r<T> &bmax, Vec3r<T> &center, unsigned int prim_index) const {
    BoundingBox(bmin, bmax, prim_index);
    const Vec3ui &face = (*faces_)[prim_index];
    center = ((*vertices_)[face[0]] + (*vertices_)[face[1]] + (*vertices_)[face[2]]) / static_cast<T>(3.);
  }
};

// Predefined SAH predicator for triangle.
template<typename T>
class TriangleSAHPred {
private:
  mutable unsigned int axis_;
  mutable T pos_;
  const Vec3rList<T> *vertices_;
  const Vec3iList *faces_;

public:
  TriangleSAHPred() = default;
  TriangleSAHPred(const Vec3rList<T> &vertices, const Vec3iList &faces)
      : axis_(0), pos_(static_cast<T>(0.0)), vertices_(&vertices), faces_(&faces) {}

  TriangleSAHPred(const TriangleSAHPred<T> &rhs)
      : axis_(rhs.axis_),
        pos_(rhs.pos_),
        vertices_(rhs.vertices_),
        faces_(rhs.faces_) {}

  TriangleSAHPred<T> &operator=(const TriangleSAHPred<T> &rhs) {
    axis_ = rhs.axis_;
    pos_ = rhs.pos_;
    vertices_ = rhs.vertices_;
    faces_ = rhs.faces_;
    return (*this);
  }

  void Set(unsigned int axis, T pos) const {
    axis_ = axis;
    pos_ = pos;
  }

  /// The operator returns true, if the triangle center is within a margin of 3 time the position along a specified axis
  inline bool operator()(unsigned int prim_id) const {

    // use precomputed center.
    // compute circumference from center and point (+margin), use this as a marker for the sah prediction
    const Vec3ui &face = (*faces_)[prim_id];
    const Vec3r<T> &p0 = (*vertices_)[face[0]];
    const Vec3r<T> &p1 = (*vertices_)[face[1]];
    const Vec3r<T> &p2 = (*vertices_)[face[2]];

    const T center = p0[axis_] + p1[axis_] + p2[axis_];

    // TODO: What the fuck ...? Where is the ```3.0``` coming from?
    // -> You could divide center by 3. -> center < pos_ ... WTF
    return (center < pos_ * static_cast<T>(3.0));
  }
};

template<typename T>
class TriangleIntersector {
public:
  const Vec3rList<T> *vertices_;
  const Vec3iList *faces_;

  mutable Vec3r<T> s;
  mutable Vec3ui k;

  mutable Vec3r<T> ray_org_;
  mutable T t_min_;
  mutable Vec2r<T> uv_;
  mutable T hit_distance;
  mutable unsigned int prim_id_;
  mutable bool cull_back_face;

public:
  inline TriangleIntersector(const Vec3rList<T> &vertices, const Vec3iList &faces)
      : vertices_(&vertices), faces_(&faces), prim_id_(-1), cull_back_face(false) {}
};

/// Update is called when initializing intersection and nearest hit is found.
template<typename T>
__attribute__((always_inline)) inline void update_intersector(TriangleIntersector<T> &i, const T t, const unsigned int prim_id) {
  i.hit_distance = t;
  i.prim_id_ = prim_id;
}

/**
   * Post BVH traversal stuff.
   * Fill `isect` if there is a hit.
   * TODO: Is Ray<T> really needed here?
   */
template<typename T>
__attribute__((always_inline)) inline void post_traversal(TriangleIntersector<T> &i, const Ray<T> &ray, const bool hit, RayHit<T> &intersection) {
  if (hit) {
    intersection.hit_distance = i.hit_distance;
    intersection.uv = i.uv_;
    intersection.prim_id = i.prim_id_;
    // TODO: Only do the normal computation if the prim is hit!
    //intersection.normal = normal_;
  }
}

/**
   * Prepare BVH traversal (e.g. compute inverse ray direction).
   * This function is called only once in BVH traversal.
   */
template<typename T>
__attribute__((always_inline))  inline void prepare_traversal(TriangleIntersector<T> &i, const Ray<T> &ray, const BVHTraceOptions<T> &trace_options) {

  i.ray_org_ = ray.origin;// copy here we'll have an allocate?
  i.t_min_ = ray.min_hit_distance;
  i.uv_ = {0., 0.};// Here happens an allocate.
  i.cull_back_face = trace_options.cull_back_face;

  Vec3ui &k = i.k;
  Vec3r<T> &s = i.s;
  const Vec3r<T> &dir = ray.direction;

  // Calculate dimension where the ray direction is maximal.
  // TODO: Vectorize this.
  k[2] = 0;
  T abs_dir = std::abs(dir[0]);

  if (abs_dir < std::abs(dir[1])) {
    k[2] = 1;
    abs_dir = std::abs(dir[1]);
  }

  if (abs_dir < std::abs(dir[2]))
    k[2] = 2;

  k[0] = k[2] + 1;
  if (k[0] == 3)
    k[0] = 0;

  k[1] = k[0] + 1;
  if (k[1] == 3)
    k[1] = 0;

  // Swap kx and ky dimension to preserve winding direction of triangles.
  if (dir[k[2]] < static_cast<T>(0.0))
    std::swap(k[1], k[2]);

  // TODO: Removed static_cast. Was it really necessary here?
  s[0] = - dir[k[0]] / dir[k[2]];
  s[1] = - dir[k[1]] / dir[k[2]];
  s[2] = static_cast<T>(1.) / dir[k[2]];
}

/**
   * Do ray intersection stuff for `prim_index` th primitive and return hit
   * distance `hit_distance`, barycentric coordinate `u` and `v`.
   * Returns true if there's intersection.
   */
template<typename T>
__attribute__((always_inline)) inline bool intersect(TriangleIntersector<T> &i, T &t_inout, const unsigned int prim_index) {

  const Vec3ui &k = i.k;
  const Vec3r<T> &s = i.s;
  const Vec3r<T> &ray_org = i.ray_org_;
  const bool cull_back_face = i.cull_back_face;

  const Vec3ui &face = (*(i.faces_))[prim_index];
  const Vec3r<T> &p0 = (*(i.vertices_))[face[0]];
  const Vec3r<T> &p1 = (*(i.vertices_))[face[1]];
  const Vec3r<T> &p2 = (*(i.vertices_))[face[2]];

  const Vec3r<T> A = p0 - ray_org;
  const Vec3r<T> B = p1 - ray_org;
  const Vec3r<T> C = p2 - ray_org;

  /**
   * TODO: These micro-optimizations need to be benchmarked.
   * precompute s such that: s[0,1] are inverted.
   * fma (-s, b, a)
   */

#ifdef BLAZERT_USE_FMA
  const T Ax = std::fma(s[0], A[k[2]], A[k[0]]);
  const T Ay = std::fma(s[1], A[k[2]], A[k[1]]);
  const T Bx = std::fma(s[0], B[k[2]], B[k[0]]);
  const T By = std::fma(s[1], B[k[2]], B[k[1]]);
  const T Cx = std::fma(s[0], C[k[2]], C[k[0]]);
  const T Cy = std::fma(s[1], C[k[2]], C[k[1]]);
#else
  const T Ax = s[0] * A[k[2]] + A[k[0]];
  const T Ay = s[1] * A[k[2]] + A[k[1]];
  const T Bx = s[0] * B[k[2]] + B[k[0]];
  const T By = s[1] * B[k[2]] + B[k[1]];
  const T Cx = s[0] * C[k[2]] + C[k[0]];
  const T Cy = s[1] * C[k[2]] + C[k[1]];
#endif

  T U = Cx * By - Cy * Bx;
  T V = Ax * Cy - Ay * Cx;
  T W = Bx * Ay - By * Ax;

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

  if (((U < static_cast<T>(0.)) | (V < static_cast<T>(0.)) | (W < static_cast<T>(0.))) && (cull_back_face | (U > static_cast<T>(0.)) | (V > static_cast<T>(0.)) | (W > static_cast<T>(0.)))) {
      return false;
  }

  const T det = U + V + W;

  if (det == static_cast<T>(0.0)) {
    return false;
  }

  const T Az = s[2] * A[k[2]];
  const T Bz = s[2] * B[k[2]];
  const T Cz = s[2] * C[k[2]];

  // TODO: This needs to be benchmarked.
#ifdef BLAZERT_USE_FMA
  const T D = std::fma(U, Az, std::fma(V, Bz, W*Cz));
#else
  const T D = U * Az + V * Bz + W * Cz;
#endif

  const T rcpDet = static_cast<T>(1.0) / det;
  const T tt = D * rcpDet;

  if ((tt > t_inout) | (tt < i.t_min_)) {
    return false;
  }

  t_inout = tt;
  /**
     * TODO: Citation needed.
     * Use Möller-Trumbore style barycentric coordinates
     * U + V + W = 1.0 and interp(p) = U * p0 + V * p1 + W * p2
     * We want interp(p) = (1 - u - v) * p0 + u * v1 + v * p2;
     * => u = V, v = W.
     */
  i.uv_ = {V * rcpDet, W * rcpDet};

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
