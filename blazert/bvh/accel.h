#pragma once
#ifndef BLAZERT_BVH_ACCEL_H_
#define BLAZERT_BVH_ACCEL_H_

#include <iostream>
#include <utility>
#include <string>
#include <atomic>
#include <limits>
#include <queue>

#include <blazert/ray.h>
#include <blazert/bvh/options.h>
#include <blazert/bvh/aabb.h>
#include <blazert/bvh/bbox.h>
#include <blazert/bvh/binbuffer.h>
#include <blazert/bvh/node.h>
#include <blazert/bvh/statistics.h>
#include <blazert/datatypes.h>
#include <blazert/defines.h>

namespace blazert {

/**
 * @brief 2-Child Bounding Volume Hierarchy (BVH).
 *
 * The BVH is a central part of ray tracing (ray traversal).
 * It takes an input geometry (primitives)  and builds a tree structure that yields
 * `O(log2 N)` complexity for ray queries (where N is the number of primitive in the scene).
 *
 * @tparam T real value type.
 */
template<typename T>
class BVH {

public:
  std::vector<BVHNode<T>> nodes;
  std::vector<unsigned int> indices;

public:
  BVH() =default;
  ~BVH() =default;

  explicit operator BVHNode<T> () const { return nodes[0]; }
  explicit operator BVHNode<T>& () const { return *(nodes[0]); }
  explicit operator BVHNode<T>* () const { return &(nodes[0]); }

  template<class Primitive, class SAH>
  BVHBuildStatistics<T> build(const Primitive &p, const SAH &sah, const BVHBuildOptions<T> &options = BVHBuildOptions<T>());

private:
  template<class Primitive, class SAH>
  unsigned int build_recursive(std::vector<BVHNode<T>> &nodes, BVHBuildStatistics<T> &statistics, const BVHBuildOptions<T> &options,
                               unsigned int left_idx, unsigned int right_idx, unsigned int depth, const Primitive &p, const SAH &sah);
};

template<typename T>
template<class Primitive, class SAH>
unsigned int BVH<T>::build_recursive(std::vector<BVHNode<T>> &nodes, BVHBuildStatistics<T> &statistics, const BVHBuildOptions<T> &options,
                                 unsigned int left_idx, unsigned int right_idx, unsigned int depth, const Primitive &p, const SAH &sah) {

  const auto offset = static_cast<unsigned int>(nodes.size());

  if (statistics.max_tree_depth < depth) statistics.max_tree_depth = depth;

  Vec3r<T> bmin, bmax;
  compute_bounding_box(bmin, bmax, indices, left_idx, right_idx, p);

  const unsigned int n = right_idx - left_idx;
  // Leaf node
  if ((n <= options.min_leaf_primitives) || (depth >= options.max_tree_depth)) {
    nodes.emplace_back();
    BVHNode<T> &node = nodes.back();
    node.min = bmin;
    node.max = bmax;

    node.leaf = 1;
    node.tris.reserve(n);

    for (unsigned int i=left_idx; i<left_idx+n ;i++) {
      const Vec3ui &f = p.faces[indices[i]];
      const Vec3rList<T> &v = p.vertices;
      node.tris.emplace_back(v[f[0]], v[f[1]], v[f[2]], indices[i]);
    }

    statistics.num_leaf_nodes++;
    return offset;
  }

  // Compute SAH and find best split axis and position
  Vec3r<T> cut_pos {0.0, 0.0, 0.0};

  BinBuffer<T> bins{options.bin_size};
  ContributeBinBuffer(bins, bmin, bmax, indices, left_idx, right_idx, p);
  int min_cut_axis = FindCutFromBinBuffer<T>(cut_pos, bins, bmin, bmax);

  // Try all 3 axis until good cut position avaiable.
  unsigned int mid_idx = left_idx;
  int cut_axis = min_cut_axis;

  for (int axis_try = 0; axis_try < 3; ++axis_try) {

    unsigned int *begin = &indices[left_idx];
    unsigned int *end = &indices[right_idx - 1] + 1;// mimics end() iterator.

    // try min_cut_axis first.
    cut_axis = (min_cut_axis + axis_try) % 3;
    sah.Set(cut_axis, cut_pos[cut_axis]);

    // Split at (cut_axis, cut_pos)
    // indices_ will be modified.
    unsigned int *mid = std::partition(begin, end, sah);

    mid_idx = left_idx + static_cast<unsigned int>((mid - begin));

    if ((mid_idx == left_idx) || (mid_idx == right_idx)) {
      // Can't split well. Switch to object median(which may create unoptimized tree, but stable)
      mid_idx = left_idx + (n >> 1);
      // Try another axis to find better cut.
    }
    else {
      // Found good cut. exit loop.
      break;
    }
  }

  // Branch node
  nodes.emplace_back();
  BVHNode<T> &node = nodes.back();
  node.axis = cut_axis;
  node.leaf = 0;

  unsigned int left_child_index  = build_recursive(nodes, statistics, options, left_idx, mid_idx, depth + 1, p, sah);
  unsigned int right_child_index = build_recursive(nodes, statistics, options, mid_idx, right_idx, depth + 1, p, sah);

  {
    //nodes[offset].children[0] = &(nodes[left_child_index]);
    //nodes[offset].children[1] = &(nodes[right_child_index]);
    nodes[offset].children[0] = left_child_index;
    nodes[offset].children[1] = right_child_index;

    nodes[offset].min = bmin;
    nodes[offset].max = bmax;
  }

  statistics.num_branch_nodes++;

  return offset;
}

template<typename T>
template<class Primitive, class SAH>
BVHBuildStatistics<T> BVH<T>::build(const Primitive &p, const SAH &sah, const BVHBuildOptions<T> &options) {

  nodes.clear();
  nodes.reserve(p.size()*2+1); // This is the absolute worst case.
  indices.resize(p.size());

#ifdef BLAZERT_PARALLEL_BUILD_OPENMP
#pragma omp parallel for
#endif
  // TODO: Here have been some static casts ... They did not seem to make sense.
  for (size_t i = 0; i < p.size(); i++) {
    indices[i] = i;
  }

  // 2. Pre-Compute bounding boxes and centers (optional).
  BVHBuildStatistics<T> statistics;
  build_recursive(nodes, statistics, options, 0, p.size(), 0, p, sah);

  indices.clear();
  indices.shrink_to_fit();
  nodes.shrink_to_fit();
  return statistics;
}

template<typename T, class I>
inline bool intersect_leaf(const BVHNode<T> &node, I &intersector, const Ray<T> &ray) {

  bool hit = false;

  for (auto &tri: node.tris) {
    hit += intersect2(intersector, tri, ray);

    if (hit && ray.any_hit) {
      break;
    }
  }
  return hit;
}

template<typename T, class I, class H>
inline bool traverse(const BVH<T> &bvh, const Ray<T> &ray, I &intersector, H &rayhit, const BVHTraceOptions<T> &options) {

  Stack<unsigned int, BLAZERT_MAX_TREE_DEPTH> node_stack;
  node_stack.push_back(0);

  T hit_distance = ray.max_hit_distance;
  prepare_traversal(intersector, ray);

  while (node_stack.size() > 0) {

    T max_hit_distance = hit_distance;
    T min_hit_distance = ray.min_hit_distance;

    const BVHNode<T> &node = bvh.nodes[node_stack.back()];
    node_stack.pop_back();

    if (intersect_node(min_hit_distance, max_hit_distance, node, ray)) {
      if (!node.leaf) { // Branch node
        const int order_near = ray.direction_sign[node.axis];
        node_stack.push_back(node.children[1 - order_near]);
        node_stack.push_back(node.children[order_near]);
      }
      /// Expect to return true, if any prim in leaf is hit and
      /// the prims hit_distance is smaller than the intersectors hit_distance.
      /// The intersector stores the closest hit_distance for all subsequent intersect_leaf queries.
      else if(intersect_leaf(node, intersector, ray)) {
        /// If a prim is hit, use this distance as max distance for all subsequent ray box intersections.
        hit_distance = intersector.hit_distance;
        if (ray.any_hit) break;
      }
    }
  }

  const bool hit = (intersector.hit_distance < ray.max_hit_distance);
  if (hit) post_traversal(intersector, rayhit);

  return hit;
}

template<typename T, typename I, typename H>
inline bool intersect_branch(T &out_max_distance, const BVHNode<T> &node, const Ray<T> &ray, I &intersector, H &rayhit) {

  // TODO: Return this distances for comparisons and subtree culling!
  T min_distance = intersector.min_hit_distance;
  T max_distance = intersector.hit_distance;

  // unlikely
  if (intersect_node(min_distance, max_distance, node, ray)) {
    // unlikely
    if (node.leaf)
      return intersect_leaf(node, intersector, ray);
    // likely
    else {
      out_max_distance = min_distance;
      return true;
    }
  }
  return false;
}

template<typename T, class I, class H>
inline bool traverse_eager(const BVH<T> &bvh, const Ray<T> &ray, I &intersector, H &rayhit, const BVHTraceOptions<T> &options) {
  prepare_traversal(intersector, ray);

  T far_max_distance, near_max_distance;
  if (!intersect_branch(far_max_distance, bvh.nodes[0], ray, intersector, rayhit)) return false;

  Stack<unsigned int, BLAZERT_MAX_TREE_DEPTH*2> node_stack;

  node_stack.push_back(0);
  unsigned int next_node = 0;

  while (node_stack.size() > 0) {

    const BVHNode<T> &node = bvh.nodes[next_node];

    const unsigned int order_near = ray.direction_sign[node.axis];
    unsigned near_idx = node.children[order_near];
    unsigned far_idx = node.children[1 - order_near];

    const BVHNode<T> &near = bvh.nodes[near_idx];
    const BVHNode<T> &far = bvh.nodes[far_idx];

    const bool hit_near = intersect_branch(near_max_distance, near, ray, intersector, rayhit);
    const bool hit_far = intersect_branch(far_max_distance, far, ray, intersector, rayhit);

    if (hit_far && hit_near && !far.leaf && !near.leaf) {
      /// Sort near and far: near is processed first.
      if (far_max_distance < near_max_distance) std::swap(far_idx, near_idx);
      next_node = near_idx;
      node_stack.push_back(far_idx);
    }
    else if (hit_far && !far.leaf) next_node = far_idx;
    else if (hit_near && !near.leaf) next_node = near_idx;
    else {
      next_node = node_stack.back();
      node_stack.pop_back();
    }
  }

  const bool hit = (intersector.hit_distance < ray.max_hit_distance);
  if (hit) post_traversal(intersector, rayhit);

  return hit;
}

template<typename T>
std::ostream& operator<<(std::ostream& stream, const BVH<T>& b) {
  // Output as JSON
  stream << "{" << std::endl; //JSON
  stream << R"("nodes": [ )" << std::endl;

  for (auto & it: b.nodes) {

    if (std::distance(begin(b.nodes), it) < b.nodes.size()-1) { std::cout << ", ";}
    else {stream << "]";}
    stream << std::endl;
  }

  stream << "}" << std::endl; //JSON
  return stream;
}

}// namespace blazert
#endif// BLAZERT_BVH_ACCEL_H_