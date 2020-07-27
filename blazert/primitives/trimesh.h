#pragma once
#ifndef BLAZERT_PRIMITIVES_TRIMESH_H_
#define BLAZERT_PRIMITIVES_TRIMESH_H_

#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <blazert/bvh/options.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/trimesh_distance.h>
#include <blazert/ray.h>

namespace blazert {

template<typename T>
struct Triangle {
  const Vec3r<T> &a;
  const Vec3r<T> b;
  const Vec3r<T> c;
  unsigned int i;
  Triangle() = delete;
  Triangle(const Vec3r<T> &a, const Vec3r<T> &b_, const Vec3r<T> &c_, const unsigned int i)
      : a(a), b(c_ - a), c(a - b_), i(i) {}
  Triangle(Triangle &&rhs) noexcept
      : a(std::move(rhs.a)), b(std::move(rhs.b)), c(std::move(rhs.c)), i(std::exchange(rhs.i, -1)) {}
  Triangle &operator=(const Triangle &rhs) = delete;
};

template<typename T, template<typename A> typename Collection,
         typename = std::enable_if_t<std::is_same<typename Collection<T>::primitive_type, Triangle<T>>::value>>
[[nodiscard]] inline Triangle<T> primitive_from_collection(const Collection<T> &collection,
                                                           const unsigned int prim_idx) {

  const Vec3ui &face = collection.faces[prim_idx];
  const Vec3r<T> &a = collection.vertices[face[0]];
  const Vec3r<T> &b = collection.vertices[face[1]];
  const Vec3r<T> &c = collection.vertices[face[2]];
  return {a, b, c, prim_idx};
}

template<typename T, template<typename A> typename Collection>
class TriangleIntersector {
public:
  const Collection<T> &collection;

  Vec3r<T> origin;
  T min_hit_distance;
  Vec2r<T> uv;
  T hit_distance;
  unsigned int prim_id;

  TriangleIntersector() = delete;
  explicit TriangleIntersector(const Collection<T> &collection)
      : collection(collection), prim_id(static_cast<unsigned int>(-1)) {}
};

template<typename T>
class TriangleMesh {

public:
  typedef TriangleIntersector<T, TriangleMesh> intersector;
  typedef Triangle<T> primitive_type;

  const Vec3rList<T> &vertices;
  const Vec3iList &faces;
  Vec3rList<T> centers;
  std::vector<std::pair<Vec3r<T>, Vec3r<T>>> box;
  Vec3rList<T> face_normals;
  Vec3rList<T> vertex_normals;

public:
  TriangleMesh() = delete;
  TriangleMesh(const TriangleMesh<T> &rhs) = delete;
  TriangleMesh(const Vec3rList<T> &vertices, const Vec3iList &faces) : vertices(vertices), faces(faces) {

    centers.reserve(faces.size());
    box.reserve(faces.size());
    face_normals.reserve(faces.size());
    vertex_normals.resize(vertices.size());

    for (auto &face : faces) {

      centers.emplace_back(pre_compute_center(face));
      box.emplace_back(pre_compute_bounding_box(face));
      face_normals.emplace_back(pre_compute_face_normal(face));

      for (auto &v : face) {
        vertex_normals[v] += face_normals.back() / static_cast<T>(3.);
      }
    }
  }

  [[nodiscard]] inline unsigned int size() const noexcept { return static_cast<unsigned int>(faces.size()); }

  [[nodiscard]] inline std::pair<Vec3r<T>, Vec3r<T>>
  get_primitive_bounding_box(const unsigned int prim_index) const noexcept {
    return box[prim_index];
  }

  [[nodiscard]] inline Vec3r<T> get_primitive_center(const unsigned int prim_index) const noexcept {
    return centers[prim_index];
  }

private:
  [[nodiscard]] inline std::pair<Vec3r<T>, Vec3r<T>> pre_compute_bounding_box(const Vec3ui &face) const noexcept {

    Vec3r<T> min = vertices[face[0]];
    Vec3r<T> max = vertices[face[0]];

    for (unsigned int i = 1; i < 3; i++) {
      const Vec3r<T> &vertex = vertices[face[i]];
      unity(min, max, vertex, vertex);
    }

    return std::make_pair(std::move(min), std::move(max));
  }

  [[nodiscard]] inline Vec3r<T> pre_compute_center(const Vec3ui &face) const noexcept {
    return (vertices[face[0]] + vertices[face[1]] + vertices[face[2]]) / static_cast<T>(3.);
  }

  [[nodiscard]] inline Vec3r<T> pre_compute_face_normal(const Vec3ui &face) const noexcept {
    const Vec3r<T> e2{vertices[face[2]] - vertices[face[0]]};
    const Vec3r<T> e1{vertices[face[0]] - vertices[face[1]]};
    return normalize(cross(e1, e2));
  }
};

template<typename T, template<typename> typename Collection>
inline void post_traversal(const TriangleIntersector<T, Collection> &i, RayHit<T> &rayhit) noexcept {
  rayhit.hit_distance = i.hit_distance;
  rayhit.uv = i.uv;
  rayhit.prim_id = i.prim_id;
  const Vec3ui &f = i.collection.faces[i.prim_id];
  // Barycentric interpolation: a =  u * p0 + v * p1 + (1-u-v) * p2;
  rayhit.normal =
      normalize((static_cast<T>(1.) - i.uv[0] - i.uv[1]) * i.collection.vertex_normals[f[0]]
                + i.uv[0] * i.collection.vertex_normals[f[1]] + i.uv[1] * i.collection.vertex_normals[f[2]]);
}

template<typename T, template<typename> typename Collection>
inline void prepare_traversal(TriangleIntersector<T, Collection> &i, const Ray<T> &ray) noexcept {
  i.min_hit_distance = ray.min_hit_distance;
  i.hit_distance = ray.max_hit_distance;
  i.uv = static_cast<T>(0.);
  i.prim_id = static_cast<unsigned int>(-1);
}

template<typename T, template<typename> typename Collection>
inline bool intersect_primitive(TriangleIntersector<T, Collection> &i, const Triangle<T> &tri,
                                const Ray<T> &ray) noexcept {
  static constexpr T tolerance = 4 * std::numeric_limits<T>::epsilon();

  const auto &e2 = tri.b;//tri.c - tri.a;
  const auto &e1 = tri.c;//tri.a - tri.b;

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
      i.hit_distance = t * inv_det;
      i.uv = {u * inv_det, v * inv_det};
      i.prim_id = tri.i;
      return true;
    }
  }
  return false;
}

template<typename T>
std::ostream &operator<<(std::ostream &stream, const TriangleMesh<T> &mesh) {
  stream << "Collection size:" << mesh.size();
  return stream;
}
}// namespace blazert
#endif// BLAZERT_PRIMITIVES_TRIMESH_H_
