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
  Vec3rList<T> centers;
  std::vector<std::pair<Vec3r<T>, Vec3r<T>>> box;
  Vec3rList<T> face_normals;
  Vec3rList<T> vertex_normals;

public:
  TriangleMesh() = delete;
  TriangleMesh(const Vec3rList<T> &vertices, const Vec3iList &faces) : vertices(vertices), faces(faces) {

    centers.reserve(faces.size());
    box.reserve(faces.size());
    face_normals.reserve(faces.size());
    vertex_normals.resize(vertices.size());

    for (auto &face : faces) {

      centers.emplace_back(compute_center(face));
      box.emplace_back(compute_bounding_box(face));
      face_normals.emplace_back(compute_face_normal(face));

      for (auto &v: face) {
        vertex_normals[v] += face_normals.back()/static_cast<T>(3.);
      }
    }
  }

  inline std::pair<Vec3r<T>, Vec3r<T>> compute_bounding_box(const Vec3ui &face) {

    Vec3r<T> min = vertices[face[0]];
    Vec3r<T> max = vertices[face[0]];

    for (unsigned int i = 1; i < 3; i++) {
      const Vec3r<T> &vertex = vertices[face[i]];
      for (int k = 0; k < 3; k++) {
        min[k] = std::min(min[k], vertex[k]);
        max[k] = std::max(max[k], vertex[k]);
      }
    }

    return {min, max};
  }

  inline Vec3r<T> compute_center(const Vec3ui &face) {
    return (vertices[face[0]] + vertices[face[1]] + vertices[face[2]]) / static_cast<T>(3.);
  }

  inline Vec3r<T> compute_face_normal(const Vec3ui &face) {
    const Vec3r<T> e2{vertices[face[2]] - vertices[face[0]]};
    const Vec3r<T> e1{vertices[face[0]] - vertices[face[1]]};
    return normalize(cross(e1, e2));
  }

  /**
   * @brief Returns the number of primitives in the triangle mesh
   * @return unsigned int
   */
  [[nodiscard]] inline unsigned int size() const { return faces.size(); }

  inline void BoundingBox(Vec3r<T> &bmin, Vec3r<T> &bmax, unsigned int prim_index) const {
    /**
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
    */
    bmin, bmax = box[prim_index];
  }

  inline void BoundingBoxAndCenter(Vec3r<T> &bmin, Vec3r<T> &bmax, Vec3r<T> &center, unsigned int prim_index) const {

    bmin, bmax = box[prim_index];
    center = centers[prim_index];
    /**
    BoundingBox(bmin, bmax, prim_index);
    const Vec3ui &face = faces[prim_index];
    center = (vertices[face[0]] + vertices[face[1]] + vertices[face[2]]) / static_cast<T>(3.);
     */
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

template<typename T, template<typename A>typename P> bool predict_sah() {}

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
