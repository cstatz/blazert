#pragma once
#ifndef BLAZERT_BVH_ACCEL_H_
#define BLAZERT_BVH_ACCEL_H_

#include <iostream>
#include <string>
#include <atomic>
#include <limits>
#include <queue>

#include <blazert/bvh/options.h>
#include <blazert/bvh/aabb.h>
#include <blazert/bvh/bbox.h>
#include <blazert/bvh/binbuffer.h>
#include <blazert/bvh/node.h>
#include <blazert/bvh/statistics.h>
#include <blazert/datatypes.h>
#include <blazert/defines.h>
#include <blazert/stack.h>

namespace blazert {

/**
 * @brief Bounding Volume Hierarchy acceleration.
 *
 * BVH is central part of ray tracing(ray traversal).
 * BVH takes an input geometry(primitive) information and build a data structure
 * for efficient ray tracing(`O(log2 N)` in theory, where N is the number of primitive in the scene).
 *
 * @tparam T real value type.
 */
template<typename T>
class BVH {

public:  // Private data
  std::vector<BVHNode<T>> nodes_;
  std::vector<unsigned int> indices_;// max 2**32 primitives.
  // TODO bboxes_ should belong to the primitives and be computed by the primitives.
  std::vector<BBox<T>> bboxes_;
  BVHBuildOptions<T> options_;
  BVHBuildStatistics stats_;

public:  // Public methods
  BVH() {}
  ~BVH() {}

  /**
   * @brief Build BVH for input primitives.
   *
   * @tparam Prim Primitive(e.g. Triangle) accessor class.
   * @tparam Pred Predicator(comparator class object for `Prim` class to find nearest hit point)
   *
   * @param[in] num_primitives The number of primitive.
   * @param[in] p Primitive accessor class object.
   * @param[in] pred Predicator object.
   *
   * @return true upon success.
   */
  template<class Prim, class Pred>
  bool build(const Prim &p, const Pred &pred, const BVHBuildOptions<T> &options = BVHBuildOptions<T>());

  /**
   * Get statistics of built BVH tree. Valid after `Build()`
   *
   * @return BVH build statistics.
   */

private:
  template<class P, class Pred>
  unsigned int build_tree(BVHBuildStatistics &out_stat, std::vector<BVHNode<T>> &out_nodes, unsigned int left_idx, unsigned int right_idx,
                         unsigned int depth, const P &p, const Pred &pred);
};

template<typename T>
template<class P, class Pred>
unsigned int BVH<T>::build_tree(BVHBuildStatistics &out_stat, std::vector<BVHNode<T>> &out_nodes, unsigned int left_idx,
                                    unsigned int right_idx, unsigned int depth, const P &p, const Pred &pred) {

  const auto offset = static_cast<unsigned int>(out_nodes.size());

  if (out_stat.max_tree_depth < depth) {
    out_stat.max_tree_depth = depth;
  }

  Vec3r<T> bmin, bmax;
  if (!bboxes_.empty()) {
    GetBoundingBox(bmin, bmax, bboxes_, indices_, left_idx, right_idx);
  }
  else {
    ComputeBoundingBox(bmin, bmax, indices_, left_idx, right_idx, p);
  }

  const unsigned int n = right_idx - left_idx;
  if ((n <= options_.min_leaf_primitives) || (depth >= options_.max_tree_depth)) {
    // Create leaf node.
    BVHNode<T> leaf;
    leaf.bmin = bmin;
    leaf.bmax = bmax;

    leaf.flag = 1;// leaf
    leaf.data[0] = n;
    leaf.data[1] = left_idx;

    out_nodes.push_back(leaf);// atomic update
    out_stat.num_leaf_nodes++;

    return offset;
  }

  //
  // Create branch node.
  //

  //
  // Compute SAH and find best split axis and position
  //
  Vec3r<T> cut_pos {0.0, 0.0, 0.0};

  BinBuffer<T> bins{options_.bin_size};
  ContributeBinBuffer(bins, bmin, bmax, indices_, left_idx, right_idx, p);
  int min_cut_axis = FindCutFromBinBuffer<T>(cut_pos, bins, bmin, bmax);

  // Try all 3 axis until good cut position avaiable.
  unsigned int mid_idx = left_idx;
  int cut_axis = min_cut_axis;

  for (int axis_try = 0; axis_try < 3; axis_try++) {

    unsigned int *begin = &indices_[left_idx];
    unsigned int *end = &indices_[right_idx - 1] + 1;// mimics end() iterator.

    // try min_cut_axis first.
    cut_axis = (min_cut_axis + axis_try) % 3;
    pred.Set(cut_axis, cut_pos[cut_axis]);

    //
    // Split at (cut_axis, cut_pos)
    // indices_ will be modified.
    //
    unsigned int *mid = std::partition(begin, end, pred);

    mid_idx = left_idx + static_cast<unsigned int>((mid - begin));

    if ((mid_idx == left_idx) || (mid_idx == right_idx)) {
      // Can't split well.
      // Switch to object median(which may create unoptimized tree, but
      // stable)
      mid_idx = left_idx + (n >> 1);
      // Try another axis to find better cut.
    }
    else {
      // Found good cut. exit loop.
      break;
    }
  }

  BVHNode<T> node;
  node.axis = cut_axis;
  node.flag = 0;  // This is a branch node

  out_nodes.push_back(node);

  unsigned int left_child_index = 0;
  unsigned int right_child_index = 0;

  // TODO: recursion is dangerous. This may yield a stack overflow.
  left_child_index =
      build_tree(out_stat, out_nodes, left_idx, mid_idx, depth + 1, p, pred);

  right_child_index =
      build_tree(out_stat, out_nodes, mid_idx, right_idx, depth + 1, p, pred);

  {
    out_nodes[offset].data[0] = left_child_index;
    out_nodes[offset].data[1] = right_child_index;

    out_nodes[offset].bmin = bmin;
    out_nodes[offset].bmax = bmax;
  }

  out_stat.num_branch_nodes++;

  return offset;
}

template<typename T>
template<class Prim, class Pred>
bool BVH<T>::build(const Prim &p, const Pred &pred, const BVHBuildOptions<T> &options) {

  options_ = options;
  stats_ = BVHBuildStatistics();
  nodes_.clear();
  bboxes_.clear();

  if (p.size() == 0) {
    return false;
  }

  unsigned int n = p.size();

  // 1. Create triangle indices(this will be permutated in build_tree)
  indices_.resize(p.size());

#ifdef BLAZERT_PARALLEL_BUILD_OPENMP
#pragma omp parallel for
#endif
  // TODO: Here have been some static casts ... They did not seem to make sense.
  for (size_t i = 0; i < n; i++) {
    indices_[i] = i;
  }

  // 2. Compute bounding box (optional).
  Vec3r<T> bmin, bmax;

  if (options.cache_bbox) {
    bmin = std::numeric_limits<T>::max();
    bmax = -std::numeric_limits<T>::max();

    bboxes_.resize(n);

    for (size_t i = 0; i < n; i++) {// for each primitive
      const unsigned int idx = indices_[i];

      BBox<T> bbox;
      p.BoundingBox(bbox.bmin, bbox.bmax, static_cast<unsigned int>(i));
      bboxes_[idx] = bbox;

      // xyz
      for (int k = 0; k < 3; k++) {
        bmin[k] = std::min(bmin[k], bbox.bmin[k]);
        bmax[k] = std::max(bmax[k], bbox.bmax[k]);
      }
    }
  }
  else {
    ComputeBoundingBox(bmin, bmax, indices_, 0, n, p);
  }

//
// 3. Build tree
//
  build_tree(stats_, nodes_, 0, n, 0, p, pred);

  return true;
}

template<typename T, class I>
inline bool test_leaf_node(const BVHNode<T> &node, I &intersector, const std::vector<unsigned int> &indices) {

  bool hit = false;

  unsigned int num_primitives = node.data[0];
  unsigned int offset = node.data[1];

  T current_hit_distance = intersector.hit_distance;// current hit distance

  for (unsigned int i = 0; i < num_primitives; i++) {
    unsigned int prim_idx = indices[i + offset];

    T local_hit_distance = current_hit_distance;
    hit = intersect(intersector, local_hit_distance, prim_idx);
    if (hit) {
      // Update isect state
      current_hit_distance = local_hit_distance;
      update_intersector(intersector, current_hit_distance, prim_idx);
    }
  }

  return hit;
}

template<typename T, class I, class H>
__attribute__((flatten)) __attribute__((always_inline)) inline bool traverse(const BVH<T> &bvh, const Ray<T> &ray, I &intersector, H &isect, const BVHTraceOptions<T> &options) {

  int node_stack_index = 0;
  unsigned int node_stack[BLAZERT_MAX_STACK_DEPTH];
  node_stack[0] = 0;

  const Vec3ui dir_sign {static_cast<unsigned int>(ray.direction[0] < static_cast<T>(0.0) ? 1 : 0),
                         static_cast<unsigned int>(ray.direction[1] < static_cast<T>(0.0) ? 1 : 0),
                         static_cast<unsigned int>(ray.direction[2] < static_cast<T>(0.0) ? 1 : 0)};

  const Vec3r<T> ray_inv_dir = static_cast<T>(1.)/ray.direction; // Check this is safe inv ..
  const Vec3r<T> &ray_org = ray.origin;

  const T num_max = std::numeric_limits<T>::max();
  T min_t = num_max;
  T max_t = -num_max;
  T hit_t = ray.max_hit_distance;

  // Init isect info as no hit
  update_intersector(intersector, hit_t, -1);
  prepare_traversal(intersector, ray, options);

  while (node_stack_index >= 0) {
    unsigned int index = node_stack[node_stack_index];
    const BVHNode<T> &node = bvh.nodes_[index];

    node_stack_index--;

    bool hit = IntersectRayAABB(min_t, max_t, ray.min_hit_distance, hit_t, node.bmin, node.bmax, ray_org, ray_inv_dir, dir_sign);

    if (hit) {
      // Branch node
      if (node.flag == 0) {
        int order_near = dir_sign[node.axis];
        int order_far = 1 - order_near;

        // Traverse near first.
        node_stack[++node_stack_index] = node.data[order_far];
        node_stack[++node_stack_index] = node.data[order_near];
      }
      else if (test_leaf_node(node, intersector, bvh.indices_)) {// Leaf node
        hit_t = intersector.hit_distance;
      }
    }
  }

  bool hit = (intersector.hit_distance < ray.max_hit_distance);
  if (hit) {
    post_traversal(intersector, ray, hit, isect);
  }
  return hit;
}

template<typename T>
std::ostream& operator<<(std::ostream& stream, const BVH<T>& b) {
  // Output as JSON
  stream << "{" << std::endl; //JSON
  stream << R"("indices": [ )" << std::endl;

  for (size_t i = 0; i < b.indices_.size(); i++) {
    stream << int(b.indices_[i]);
    if (i < b.indices_.size()-1) { std::cout << ", ";}
  }

  stream << "], " << std::endl;
  stream << R"("nodes": [ )" << std::endl;

  for (size_t i = 0; i < b.nodes_.size(); i++) {
    stream << "{";
    stream << R"("bmin": [)";
    for (int kk=0; kk<3; kk++) {
      stream << b.nodes_[i].bmin[kk];
      if (kk < 2) { std::cout << ", ";}
    }
    stream << R"(], "bmax": [)";
    for (int kk=0; kk<3; kk++) {
      stream << b.nodes_[i].bmax[kk];
      if (kk < 2) { std::cout << ", ";}
    }
    stream << "]}";
    if (i < b.nodes_.size()-1) { std::cout << ", ";}
    else {std::cout << "]";}
    stream << std::endl;
  }

  stream << "}" << std::endl; //JSON
  return stream;
}

}// namespace blazert

#endif// BLAZERT_BVH_ACCEL_H_
