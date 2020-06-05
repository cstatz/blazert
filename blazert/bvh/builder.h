//
// Created by Christoph Statz on 02.06.20.
//
#pragma once
#ifndef BLAZERT_BLAZERT_BVH_BUILDER_H
#define BLAZERT_BLAZERT_BVH_BUILDER_H

#include <iostream>
#include <utility>
#include <string>
#include <atomic>
#include <limits>
#include <queue>

#include <blazert/ray.h>
#include <blazert/bvh/options.h>
#include <blazert/bvh/intersect.h>
#include <blazert/bvh/bbox.h>
#include <blazert/bvh/binbuffer.h>
#include <blazert/bvh/node.h>
#include <blazert/bvh/statistics.h>
#include <blazert/datatypes.h>
#include <blazert/defines.h>

namespace blazert {

template<typename T, template<typename> typename Collection>
struct SAH {
  const unsigned int axis;
  const T position;
  const Collection<T> &collection;
  SAH(const Collection<T> &collection, const unsigned int axis, const T position) : axis(axis), position(position), collection(collection) {};
  inline bool operator()(const unsigned int prim_id) const {
    return predict_sah(collection, prim_id, axis, position);
  }
};

//template<typename T, template<typename> typename Collection, template<typename, template<typename> typename> typename BVH>
class SAHBinnedBuilder {
public:
  // TODO: statistics and options might be specific to builder type.
  std::vector<unsigned int> indices;

  SAHBinnedBuilder() = default;

  template<typename T, template<typename> typename Collection, template<typename, template<typename> typename> typename BVH>
  BVHBuildStatistics<T> build(BVH<T, Collection> &bvh, const BVHBuildOptions<T> &options = BVHBuildOptions<T>()) {

    BVHBuildStatistics<T> statistics;
    statistics.start();

    indices.clear();

    // TODO: catch bad_alloc
    indices.resize(bvh.collection.size());

#ifdef BLAZERT_PARALLEL_BUILD_OPENMP
#pragma omp parallel for
#endif
    for (unsigned int i = 0; i < bvh.collection.size(); i++) indices[i] = i;

    // TODO: catch bad_alloc
    bvh.nodes.reserve(2 * bvh.collection.size());

    build_recursive(bvh.nodes, indices, statistics, options, 0, bvh.collection.size(), 0, bvh.collection);

    bvh.nodes.shrink_to_fit();
    indices.clear();
    indices.shrink_to_fit();

    statistics.stop();
    return statistics;
  }
};

template<typename T, template<typename> typename Collection>
unsigned int build_recursive(std::vector<BVHNode<T, Collection>> &nodes, std::vector<unsigned int> &indices,
                             BVHBuildStatistics<T> &statistics, const BVHBuildOptions<T> &options,
                             const unsigned int left_idx, const unsigned int right_idx, const unsigned int depth, const Collection<T> &collection) {

  const auto offset = static_cast<unsigned int>(nodes.size());

  if (statistics.max_tree_depth < depth) statistics.max_tree_depth = depth;

  Vec3r<T> bmin, bmax;
  compute_bounding_box(bmin, bmax, indices, left_idx, right_idx, collection);

  const unsigned int n = right_idx - left_idx;
  // Leaf node
  if ((n <= options.min_leaf_primitives) || (depth >= options.max_tree_depth)) {
    BVHNode<T, Collection> node;
    node.min = bmin;
    node.max = bmax;

    node.leaf = 1;
    node.primitives.reserve(n);

    for (unsigned int i = left_idx; i < left_idx + n; i++) {
      // Primitives need to be move constructable
      node.primitives.push_back(std::move(primitive_from_collection(collection, indices[i])));
    }

    statistics.primitives_per_leaf(n);
    statistics.leaf_nodes++;
    nodes.push_back(std::move(node));
    return offset;
  }

  // Compute SAH and find best split axis and position
  Vec3r<T> cut_pos{0.0, 0.0, 0.0};

  BinBuffer<T> bins{options.bin_size};
  contribute_bins(bins, bmin, bmax, indices, left_idx, right_idx, collection);
  int min_cut_axis = find_cut_from_bins(cut_pos, bins, bmin, bmax);

  // Try all 3 axis until good cut position available.
  unsigned int mid_idx = left_idx;
  unsigned int cut_axis = min_cut_axis;

  for (unsigned int axis_try = 0; axis_try < 3; ++axis_try) {
    unsigned int *begin = &indices[left_idx];
    unsigned int *end = &indices[right_idx - 1] + 1;// mimics end() iterator.

    // try min_cut_axis first.
    cut_axis = (min_cut_axis + axis_try) % 3;

    const SAH sah(collection, cut_axis, cut_pos[cut_axis]);

    // Split at (cut_axis, cut_pos)
    // indices will be modified.
    unsigned int *mid = std::partition(begin, end, sah);

    mid_idx = left_idx + static_cast<unsigned int>((mid - begin));

    if ((mid_idx == left_idx) || (mid_idx == right_idx)) {
      // Can't split well. Switch to object median(which may create unoptimized tree, but stable)
      statistics.bad_splits++;
      mid_idx = left_idx + (n >> 1);
      // Try another axis to find better cut.
    } else {
      // Found good cut. exit loop.
      break;
    }
  }

  // Branch node
  BVHNode<T, Collection> node;
  node.axis = cut_axis;
  node.leaf = 0;

  nodes.push_back(std::move(node));

  const unsigned int left_child_index = build_recursive(nodes, indices, statistics, options, left_idx, mid_idx, depth + 1, collection);
  const unsigned int right_child_index = build_recursive(nodes, indices, statistics, options, mid_idx, right_idx, depth + 1, collection);

  nodes[offset].children[0] = left_child_index;
  nodes[offset].children[1] = right_child_index;
  nodes[offset].min = bmin;
  nodes[offset].max = bmax;

  statistics.branch_nodes++;

  return offset;
}
}
#endif//BLAZERT_BLAZERT_BVH_BUILDER_H
