#pragma once
#ifndef BLAZERT_BVH_ACCEL_H_
#define BLAZERT_BVH_ACCEL_H_

#include <atomic>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <utility>

#include <blazert/bvh/intersect.h>
#include <blazert/bvh/node.h>
#include <blazert/datatypes.h>
#include <blazert/defines.h>
#include <blazert/ray.h>

namespace blazert {

/**
 * @brief 2-Child Bounding Volume Hierarchy (BVH).
 *
 * The BVH is a central part of ray tracing (bvh traversal).
 * It takes an input geometry (primitive collection) and builds a tree data-structure that yields
 * `O(log2 N)` complexity for ray queries (where N is the number of primitive in the collection).
 *
 * @tparam T real value type.
 * @tparam Primitive primitive collection
 * @tparam Builder tree builder
 */
template<typename T, template<typename> typename Collection>
class BVH {

public:
  std::vector<BVHNode<T, Collection>> nodes;
  const Collection<T> &collection;// TODO: maybe shared_ptr or something else. Otherwise usage in scene is not trivial.

public:
  BVH() = delete;
  BVH(const BVH &rhs) = delete;
  explicit BVH(const Collection<T> &collection) : collection(collection) {}
  ~BVH() = default;

  // TODO: Implement move constructor
  explicit operator BVHNode<T, Collection>() const { return nodes[0]; }
  explicit operator BVHNode<T, Collection> &() const { return *(nodes[0]); }
  explicit operator BVHNode<T, Collection> *() const { return &(nodes[0]); }
};

template<typename T, template<typename> typename Collection>
inline bool traverse(const BVH<T, Collection> &bvh, const Ray<T> &ray, RayHit<T> &rayhit) noexcept {
  typename Collection<T>::intersector intersector(bvh.collection);
  return traverse(bvh, ray, rayhit, intersector);
}

template<typename T, template<typename> typename Collection>
inline bool traverse(const BVH<T, Collection> &bvh, const Ray<T> &ray, RayHit<T> &rayhit,
                     typename Collection<T>::intersector &intersector) noexcept {

  Stack<unsigned int, BLAZERT_MAX_TREE_DEPTH> node_stack;
  node_stack.push_back(0);

  T hit_distance = ray.max_hit_distance;
  prepare_traversal(intersector, ray);

  while (node_stack.size() > 0) {

    T max_hit_distance = hit_distance;
    T min_hit_distance = ray.min_hit_distance;

    const BVHNode<T, Collection> &node = bvh.nodes[node_stack.back()];
    node_stack.pop_back();

    if (intersect_node(min_hit_distance, max_hit_distance, node, ray)) {
      if (!node.leaf) {// Branch node
        const unsigned int order_near = ray.direction_sign[node.axis];
        node_stack.push_back(node.children[1 - order_near]);
        node_stack.push_back(node.children[order_near]);
      }
      /// Expect to return true, if any prim in leaf is hit and
      /// the prims hit_distance is smaller than the intersectors hit_distance.
      /// The intersector stores the closest hit_distance for all subsequent intersect_leaf queries.
      else if (intersect_leaf(node, intersector, ray)) {
        /// If a prim is hit, use this distance as max distance for all subsequent ray box intersections.
        hit_distance = intersector.hit_distance;
        if (ray.any_hit)
          break;
      }
    }
  }

  const bool hit = (intersector.hit_distance < ray.max_hit_distance);
  if (hit)
    post_traversal(intersector, rayhit);

  return hit;
}

template<typename T, template<typename> typename Primitive>
std::ostream &operator<<(std::ostream &stream, const BVH<T, Primitive> &b) {
  /// Conveniently output bvh as JSON.
  stream << "{"
         << "\n";
  stream << R"("nodes": [ )"
         << "\n";

  for (auto &it : b.nodes) {
    stream << it;

    if (&it - &*(b.nodes.begin()) < b.nodes.size() - 1) {
      stream << ", ";
    } else {
      stream << "]";
    }
    stream << "\n";
  }

  stream << "}"
         << "\n";//JSON
  return stream;
}

}// namespace blazert
#endif// BLAZERT_BVH_ACCEL_H_
