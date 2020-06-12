//
// Created by Christoph Statz on 02.06.20.
//
#pragma once
#ifndef BLAZERT_BLAZERT_BVH_BUILDER_H
#define BLAZERT_BLAZERT_BVH_BUILDER_H

#include <atomic>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <utility>

#include <blazert/bvh/bbox.h>
#include <blazert/bvh/binbuffer.h>
#include <blazert/bvh/intersect.h>
#include <blazert/bvh/node.h>
#include <blazert/bvh/options.h>
#include <blazert/bvh/statistics.h>
#include <blazert/datatypes.h>
#include <blazert/defines.h>
#include <blazert/ray.h>

namespace blazert {

class SAHBinnedBuilder {
public:
  // TODO: statistics and options might be specific to builder type.
  std::vector<unsigned int> indices;

  SAHBinnedBuilder() = default;

  template<typename T, template<typename> typename Collection,
           template<typename, template<typename> typename> typename BVH>
  BVHBuildStatistics<T> build(BVH<T, Collection> &bvh, const BVHBuildOptions<T> &options = BVHBuildOptions<T>()) {

    BVHBuildStatistics<T> statistics;
    statistics.start();

    indices.resize(bvh.collection.size());
    std::iota(indices.begin(), indices.end(), 0);

    bvh.nodes.reserve(2 * bvh.collection.size());
    build_recursive(bvh.nodes, bvh.collection, indices.begin(), indices.end(), 0, statistics, options);

    bvh.nodes.shrink_to_fit();
    indices.clear();
    indices.shrink_to_fit();

    statistics.stop();
    return statistics;
  }
};

template<typename T, typename Iterator, template<typename> typename Collection>
unsigned int build_recursive(std::vector<BVHNode<T, Collection>> &nodes, const Collection<T> &collection,
                             Iterator begin, Iterator end, const unsigned int depth, BVHBuildStatistics<T> &statistics,
                             const BVHBuildOptions<T> &options) {

  const auto offset = static_cast<unsigned int>(nodes.size());

  if (statistics.max_tree_depth < depth)
    statistics.max_tree_depth = depth;

  const unsigned int n = std::distance(begin, end);
  const auto [min, max] = compute_bounding_box<T>(collection, begin, end);

  // Leaf
  if ((n <= options.min_leaf_primitives) || (depth >= options.max_tree_depth)) {
    nodes.push_back(std::move(create_leaf(collection, begin, end, min, max)));
    statistics.primitives_per_leaf(n);
    statistics.leaf_nodes++;
    return offset;
  }

  // Branch
  auto [branch, mid] = create_branch(collection, begin, end, min, max, statistics, options);
  nodes.push_back(std::move(branch));
  statistics.branch_nodes++;

  nodes[offset].children[0] = build_recursive(nodes, collection, begin, mid, depth + 1, statistics, options);
  nodes[offset].children[1] = build_recursive(nodes, collection, mid, end, depth + 1, statistics, options);

  return offset;
}

template<typename T, typename Iterator, template<typename> typename Collection>
inline BVHNode<T, Collection> create_leaf(const Collection<T> &collection, Iterator begin, Iterator end,
                                          const Vec3r<T> &bmin, const Vec3r<T> &bmax) {
  BVHNode<T, Collection> node;
  node.min = bmin;
  node.max = bmax;

  node.leaf = 1;
  node.primitives.reserve(std::distance(begin, end));

  for (auto it = begin; it != end; ++it) {
    node.primitives.push_back(std::move(primitive_from_collection(collection, *it)));
  }

  return node;
};

template<typename T, typename Iterator, template<typename> typename Collection>
inline std::pair<BVHNode<T, Collection>, Iterator>
create_branch(const Collection<T> &collection, Iterator begin, Iterator end, const Vec3r<T> &bmin, const Vec3r<T> &bmax,
              BVHBuildStatistics<T> &statistics, const BVHBuildOptions<T> &options) {

  const auto pair = split(collection, begin, end, bmin, bmax, statistics, options);
  const auto &axis = pair.first;
  const auto &mid = pair.second;

  BVHNode<T, Collection> node;
  node.leaf = 0;
  node.min = bmin;
  node.max = bmax;
  node.axis = axis;

  return std::make_pair(std::move(node), std::move(mid));
}

template<typename T, typename Iterator, template<typename> typename Collection>
inline std::pair<unsigned int, Iterator> split(const Collection<T> &collection, Iterator begin, Iterator end,
                                               const Vec3r<T> &bmin, const Vec3r<T> &bmax,
                                               BVHBuildStatistics<T> &statistics, const BVHBuildOptions<T> &options) {

  auto pair = find_best_split_binned(collection, begin, end, bmin, bmax, options);
  auto cut_axis = pair.first;
  const auto &cut_pos = pair.second;
  Iterator mid;

  for (unsigned int axis_try = 0; axis_try < 3; axis_try++) {

    mid = std::partition(begin, end, [&collection, cut_axis, &cut_pos](const auto it) {
      return collection.get_primitive_center(it)[cut_axis] < cut_pos[cut_axis];
    });

    if ((std::distance(begin, mid) == 0) || (std::distance(mid, end) == 0)) {
      statistics.bad_splits++;
      mid = begin + (std::distance(begin, end) >> 1);
      cut_axis++;
      cut_axis %= 3;
    } else
      break;
  }
  return std::make_pair(cut_axis, mid);
}
}// namespace blazert
#endif//BLAZERT_BLAZERT_BVH_BUILDER_H
