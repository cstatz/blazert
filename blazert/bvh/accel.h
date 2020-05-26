#pragma once
#ifndef BLAZERT_BVH_ACCEL_H_
#define BLAZERT_BVH_ACCEL_H_

#include <iostream>
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
 * It takes an input geometry (primitivse)  and builds a tree structure that yields
 * `O(log2 N)` complexity for ray queries (where N is the number of primitive in the scene).
 *
 * @tparam T real value type.
 */
template<typename T>
class BVH {

public:  // Private data
  std::vector<BVHNode<T>> nodes;
  std::vector<unsigned int> indices; // max 2**32 primitives.

public:  // Public methods
  BVH() {}
  ~BVH() {}

  template<class Primitive, class SAH>
  BVHBuildStatistics build(const Primitive &p, const SAH &sah, const BVHBuildOptions<T> &options = BVHBuildOptions<T>());


private:
  template<class Primitive, class SAH>
  unsigned int build_recursive(std::vector<BVHNode<T>> &nodes, BVHBuildStatistics &statistics, const BVHBuildOptions<T> &options,
                               unsigned int left_idx, unsigned int right_idx, unsigned int depth, const Primitive &p, const SAH &sah);
};

template<typename T>
template<class Primitive, class SAH>
unsigned int BVH<T>::build_recursive(std::vector<BVHNode<T>> &nodes, BVHBuildStatistics &statistics, const BVHBuildOptions<T> &options,
                                 unsigned int left_idx, unsigned int right_idx, unsigned int depth, const Primitive &p, const SAH &sah) {

  const auto offset = static_cast<unsigned int>(nodes.size());

  if (statistics.max_tree_depth < depth) {
    statistics.max_tree_depth = depth;
  }

  Vec3r<T> bmin, bmax;
  compute_bounding_box(bmin, bmax, indices, left_idx, right_idx, p);

  const unsigned int n = right_idx - left_idx;
  if ((n <= options.min_leaf_primitives) || (depth >= options.max_tree_depth)) {
    // Create leaf node
    BVHNode<T> leaf;
    leaf.min = bmin;
    leaf.max = bmax;

    leaf.flag = 1;  // leaf
    leaf.data[0] = n;  // size
    leaf.data[1] = left_idx;  // left_index (offset later on)
    // TODO: Prebuild elementary prims, if requested (might accelerate tracing at cost of memory).
    nodes.push_back(leaf);  // atomic update
    statistics.num_leaf_nodes++;

    return offset;
  }

  //
  // Create branch node.
  //

  //
  // Compute SAH and find best split axis and position
  //
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

    //
    // Split at (cut_axis, cut_pos)
    // indices_ will be modified.
    //
    unsigned int *mid = std::partition(begin, end, sah);

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

  nodes.push_back(node);

  unsigned int left_child_index = 0;
  unsigned int right_child_index = 0;

  // TODO: recursion is dangebuild_treerous. This may yield a stack overflow.
  left_child_index  = build_recursive(nodes, statistics, options, left_idx, mid_idx, depth + 1, p, sah);
  right_child_index = build_recursive(nodes, statistics, options, mid_idx, right_idx, depth + 1, p, sah);

  {
    nodes[offset].data[0] = left_child_index;
    nodes[offset].data[1] = right_child_index;

    nodes[offset].min = bmin;
    nodes[offset].max = bmax;
  }

  statistics.num_branch_nodes++;

  return offset;
}

template<typename T>
template<class Primitive, class SAH>
BVHBuildStatistics BVH<T>::build(const Primitive &p, const SAH &sah, const BVHBuildOptions<T> &options) {

  nodes.clear();
  indices.resize(p.size());

#ifdef BLAZERT_PARALLEL_BUILD_OPENMP
#pragma omp parallel for
#endif
  // TODO: Here have been some static casts ... They did not seem to make sense.
  for (size_t i = 0; i < p.size(); i++) {
    indices[i] = i;
  }

  // 2. Pre-Compute bounding boxes and centers (optional).
  BVHBuildStatistics statistics;
  build_recursive(nodes, statistics, options, 0, p.size(), 0, p, sah);

  return statistics;
}


template<typename T, class I>
inline bool intersect_leaf(const BVHNode<T> &node, I &intersector, const std::vector<unsigned int> &indices) {

  bool hit = false;

  const unsigned int num_primitives = node.data[0];
  const unsigned int offset = node.data[1];

  T current_hit_distance = intersector.hit_distance;// current hit distance

  for (unsigned int i = 0; i < num_primitives; i++) {
    const unsigned int prim_idx = indices[i + offset];

    // TODO: The design is sh****.
    // node -> offset, num_prims -> indices -> face (from faces) -> vertex (from vertices)
    // 5 indirections
    // Better alternative: node -> primitives -> vertices
    // build-time + memory vs. traversal speed.

    // slightly larger node -> store pointers to child node and

    T local_hit_distance = current_hit_distance;
    hit = intersect(intersector, local_hit_distance, prim_idx);

    if (hit) {
      current_hit_distance = local_hit_distance;
      update_intersector(intersector, current_hit_distance, prim_idx);
#ifdef BLAZERT_FIRST_INSTEAD_OF_CLOSEST
      break; // Any intersection will do.
#endif
    }
  }

  return hit;
}

template<typename T, class I, class H>
inline bool traverse(const BVH<T> &bvh, const Ray<T> &ray, I &intersector, H &rayhit, const BVHTraceOptions<T> &options) {

  int node_stack_index = 0;
  unsigned int node_stack[BLAZERT_MAX_STACK_DEPTH];

  node_stack[node_stack_index] = 0;

  T hit_t = ray.max_hit_distance;

  update_intersector(intersector, ray.max_hit_distance, -1);
  prepare_traversal(intersector, ray, options);

  while (node_stack_index >= 0) {
    const unsigned int index = node_stack[node_stack_index--];
    const BVHNode<T> &node = bvh.nodes[index];

    T max_t = hit_t;
    //T max_t = ray.max_hit_distance;
    T min_t = ray.min_hit_distance;

    if (intersect_node(min_t, max_t, node, ray)) {
      // Branch node
      if (!node.flag) {
        const int order_near = ray.direction_sign[node.axis];
        const int order_far = 1 - order_near;

        // Traverse near first.
        node_stack[++node_stack_index] = node.data[order_far];
        node_stack[++node_stack_index] = node.data[order_near];

      }
      // Leaf node
      else if (intersect_leaf(node, intersector, bvh.indices)) {
        //intersect_leaf(node, intersector, bvh.indices);
        hit_t = intersector.hit_distance;
      }
    }
  }

  const bool hit = (intersector.hit_distance < ray.max_hit_distance);
  if (hit) {
    post_traversal(intersector, ray, hit, rayhit);
  }
  return hit;
}

template<typename T, typename I, typename H, bool first_node=false>
inline bool intersect_branch(const BVH<T> &bvh, const BVHNode<T> &node, const Ray<T> &ray, const BVHTraceOptions<T> &options, I &intersector, H &isect) {
  // Test the first node (box and if necessary leaf)
  constexpr T num_max = std::numeric_limits<T>::max();
  T min_distance = num_max;
  T max_distance = -num_max;
  const T hit_distance = ray.max_hit_distance;

  //const BVHNode<T> &node = bvh.nodes_[0];
  // unlikely
  if (!intersect_node2(min_distance, max_distance, hit_distance, node, ray))
    return false; // Exit early if ray misses the base node

  update_intersector(intersector, hit_distance, -1);

  if constexpr(first_node) {
    prepare_traversal(intersector, ray, options);
  }

  // unlikely
  if (node.flag) {  // 1 -> leaf node
    const bool hit = intersect_leaf(node, intersector, bvh.indices);
    if (hit) {
      post_traversal(intersector, ray, hit, isect);
    }
    // Exit early
    return hit;
  }
}
//
//template<typename T, class I, class H>
//inline bool traverse_eager(const BVH<T> &bvh, const Ray<T> &ray, I &intersector, H &isect, const BVHTraceOptions<T> &options) {
//
//  const Vec3ui dir_sign{static_cast<unsigned int>(ray.direction[0] < static_cast<T>(0.0) ? 1 : 0),
//                        static_cast<unsigned int>(ray.direction[1] < static_cast<T>(0.0) ? 1 : 0),
//                        static_cast<unsigned int>(ray.direction[2] < static_cast<T>(0.0) ? 1 : 0)};
//
//  // This is expensive ...
//  const Vec3r<T> ray_inv_dir = static_cast<T>(1.) / ray.direction;// Check this is safe inv ..
//  const Vec3r<T> &ray_org = ray.origin;
//
//
//  // TODO: DRY this scope up.
//  if (!intersect_branch<T, I, H, true>()) return false;
//
//
//  int node_stack_index = 0;
//  unsigned int node_stack[BLAZERT_MAX_STACK_DEPTH];
//  node_stack[0] = 0;
//
//  while (node_stack_index >= 0) {
//
//    /// Get the first node from the node stack index. This node has already been tested to be hit by the ray
//    const unsigned int index = node_stack[node_stack_index];
//    const BVHNode<T> &node = bvh.nodes_[index];
//    node_stack_index--;
//
//    /// Get the child nodes and intersect them. It they hit and if they are leaf node, intersect the childs (test the closest first).
//    const unsigned int order_near = dir_sign[node.axis];
//    const unsigned int order_far = 1 - order_near;
//
//    const auto &near_child = bvh.nodes_[node.data[order_near]];
//    /// Test near child:
//
//    intersect_branch();
//
//    const bool hit_left = intersect_node(min_t, max_t, ray.min_hit_distance, hit_t, node.min, node.max, ray_org, ray_inv_dir, dir_sign);
//    if (hit_left) {}
//
//
//    const auto &far_child = bvh.nodes_[node.data[order_far]];
//    /// Test far child:
//    {
//
//    }
//
//    /// If there is a prim hit: return/break fast (this only traces the closest hit)
//
//    /// Put the closest branch-node on the node stack (and cull the far branch).
//
//    if (hit) {
//      // Branch node
//      if (!node.flag) {
//        const int order_near = dir_sign[node.axis];
//        const int order_far = 1 - order_near;
//
//        // Traverse near first.
//        node_stack[++node_stack_index] = node.data[order_far];
//        node_stack[++node_stack_index] = node.data[order_near];
//      }
//        // Leaf node
//      else if (intersect_leaf(node, intersector, bvh.indices_)) {// Leaf node
//        hit_t = intersector.hit_distance;
//      }
//    }
//  }
//
//  const bool hit = (intersector.hit_distance < ray.max_hit_distance);
//  if (hit) {
//    post_traversal(intersector, ray, hit, isect);
//  }
//  return hit;
//}

//template<typename T, class I, class H>
//inline bool traverse_eager(const BVH<T> &bvh, const Ray<T> &ray, I &intersector, H &isect, const BVHTraceOptions<T> &options)  {
//
//  // Test leaf and exit early if the root is a leaf.
//  if (bvh.nodes_[0].flag) {
//    const bool hit = intersect_leaf(bvh.nodes_[0], intersector, bvh.indices_);
//    if (hit) {
//      post_traversal(intersector, ray, hit, isect);
//    }
//    return hit;
//  }
//
//  // Pre-compute ray specific
//  const Vec3ui dir_sign {static_cast<unsigned int>(ray.direction[0] < static_cast<T>(0.0) ? 1 : 0),
//                         static_cast<unsigned int>(ray.direction[1] < static_cast<T>(0.0) ? 1 : 0),
//                         static_cast<unsigned int>(ray.direction[2] < static_cast<T>(0.0) ? 1 : 0)};
//
//  const Vec3r<T> ray_inv_dir = static_cast<T>(1.)/ray.direction; // Check this is safe inv ..
//  const Vec3r<T> &ray_org = ray.origin;
//
//  const T num_max = std::numeric_limits<T>::max();
//  T min_t_l = num_max;
//  T max_t_l = -num_max;
//  T min_t_r = num_max;
//  T max_t_r = -num_max;
//  T hit_t_l = ray.max_hit_distance;
//  T hit_t_r = ray.max_hit_distance;
//
//  // Init isect info as no hit_
//  update_intersector(intersector, hit_t_l, -1);
//  prepare_traversal(intersector, ray, options);
//
//  /// Test node 0 before node stack -> exit early
//
//  int node_stack_index = 0;
//  unsigned int node_stack[BLAZERT_MAX_STACK_DEPTH];
//  node_stack[0] = 0;
//
//  while (true) {
//
//    const auto &node = bvh.nodes_[node_stack[node_stack_index--]];
//    /// Get the child nodes, test the child nodes (only if node is a branch)
//    if (!node.flag) {
//      const auto &left_child = bvh.nodes_[node.data[0]];
//      const auto &right_child = bvh.nodes_[node.data[1]];
//
//      // Check if node is leaf or branch before aabb testing -> test the lead bb
//      const bool hit_left = intersect_node(min_t_l, max_t_l, ray.min_hit_distance, hit_t_l, node.min, node.max, ray_org, ray_inv_dir, dir_sign);
//      // Assumptions --> childs ordered in a manner that the left child is closed
//      // first, second -> tmin, tmax
//      if (hit_left && left_child.flag) {// If left leaf node: test prims
//        update_intersector(intersector, hit_t_l, -1);
//        const bool hit = intersect_leaf(left_child, intersector, bvh.indices_);
//        if (hit && (intersector.hit_distance < ray.max_hit_distance)) {
//          post_traversal(intersector, ray, hit, isect);
//          return hit;
//        }
//      }
//
//      const bool hit_right = intersect_node(min_t_r, max_t_r, ray.min_hit_distance, hit_t_r, node.min, node.max, ray_org, ray_inv_dir, dir_sign);
//      if (hit_right && right_child.flag) {// If right leaf node: test prims
//        update_intersector(intersector, hit_t_r, -1);
//        const bool hit = intersect_leaf(right_child, intersector, bvh.indices_);
//        if (hit && (intersector.hit_distance < ray.max_hit_distance)) {
//          post_traversal(intersector, ray, hit, isect);
//          return hit;
//        }
//      }
//
//      if (!(hit_left || hit_right)) {// not left, not right, not both
//        // update node_idx from stack, if stack is empty ... -> break
//        if (node_stack_index < 0)
//          return false;
//        // In the while loop take the next element from the node stack.
//      } else {// right, left or both
//        if (hit_left && hit_right) {
//          node_stack[++node_stack_index] = (min_t_l < min_t_r) ? node.data[0] : node.data[1];
//        } else {
//          if (hit_left) {
//            node_stack[++node_stack_index] = node.data[0];
//          } else {
//            node_stack[++node_stack_index] = node.data[1];
//          }
//        }
//      }
//    }
//  }
//}

template<typename T>
std::ostream& operator<<(std::ostream& stream, const BVH<T>& b) {
  // Output as JSON
  stream << "{" << std::endl; //JSON
  stream << R"("indices": [ )" << std::endl;

  for (size_t i = 0; i < b.indices.size(); i++) {
    stream << int(b.indices[i]);
    if (i < b.indices.size()-1) { std::cout << ", ";}
  }

  stream << "], " << std::endl;
  stream << R"("nodes": [ )" << std::endl;

  for (size_t i = 0; i < b.nodes.size(); i++) {
    stream << "{";
    stream << R"("min": [)";
    for (int kk=0; kk<3; kk++) {
      stream << b.nodes[i].min[kk];
      if (kk < 2) { std::cout << ", ";}
    }
    stream << R"(], "max": [)";
    for (int kk=0; kk<3; kk++) {
      stream << b.nodes[i].max[kk];
      if (kk < 2) { std::cout << ", ";}
    }
    stream << "]}";
    if (i < b.nodes.size()-1) { std::cout << ", ";}
    else {std::cout << "]";}
    stream << std::endl;
  }

  stream << "}" << std::endl; //JSON
  return stream;
}

}// namespace blazert

#endif// BLAZERT_BVH_ACCEL_H_
