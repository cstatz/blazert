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

  /**
   * This function builds the BVH with the specified BVHBuildOptions. If no BVHBuildOptions are specified the standard
   * build options are used.
   *
   * @tparam T floating point type
   * @tparam Collection primitive collection type
   * @tparam BVH BVH type
   * @param bvh BVH type in which the build BVH is stored
   * @param options BVHBuildOptions
   * @return BVHBuildStatistics structure holding data about the build process.
   */
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

/**
 * Recursive build function to build the BVH from a collection of primitives.
 * @tparam T floating point type
 * @tparam Iterator Iterator type
 * @tparam Collection primitive collection type
 *
 * @param nodes std::vector of BVHNodes to which the built nodes are attached
 * @param collection collection of primitives which need to be stored in the BVH
 * @param begin iterator to the first element to be considered for further construction of the BVH
 * @param end iterator to the last element to be considered for further construction of the BVH
 * @param depth current depth of the BVH
 * @param statistics BVHBuildStatistics structure to keep track of the build
 * @param options Build options
 *
 * @return offset/index into nodes for the current BVHNode
 */
template<typename T, typename Iterator, template<typename> typename Collection>
unsigned int build_recursive(std::vector<BVHNode<T, Collection>> &nodes, const Collection<T> &collection,
                             Iterator begin, Iterator end, const unsigned int depth, BVHBuildStatistics<T> &statistics,
                             const BVHBuildOptions<T> &options) {

  const auto offset = static_cast<unsigned int>(nodes.size());

  if (statistics.max_tree_depth < depth)
    statistics.max_tree_depth = depth;

  // std::distance theoretically can be negative, but it never is in this case
  const unsigned int n = static_cast<unsigned int>(std::distance(begin, end));
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

/**
 * This functions creates a leaf node (= node cannot have children) for the BVH containing the primitves from the
 * collection within the iterator range [begin; end]
 *
 * @tparam T floating point type
 * @tparam Iterator Iterator type
 * @tparam Collection Collection type
 *
 * @param collection collection of primitives
 * @param begin iterator specifying the beginning of the primitives from the collection to insert in the leaf
 * @param end iterator specifying the end of the primitives from the collection to insert in the leaf
 * @param bmin vertex describing the minimum extent of the bounding box
 * @param bmax vertex describing the maximum extent of the bounding box
 *
 * @return BVHNode
 */
template<typename T, typename Iterator, template<typename> typename Collection>
inline BVHNode<T, Collection> create_leaf(const Collection<T> &collection, Iterator begin, Iterator end,
                                          const Vec3r<T> &bmin, const Vec3r<T> &bmax) {
  BVHNode<T, Collection> node;
  node.min = bmin;
  node.max = bmax;

  node.leaf = 1;
  node.primitives.reserve(static_cast<long unsigned int>(std::distance(begin, end)));

  for (auto it = begin; it != end; ++it) {
    node.primitives.push_back(std::move(primitive_from_collection(collection, *it)));
  }

  return node;
}

/**
 * This function creates a branch node of the BVH which has two children which can either be leaf or branch nodes.
 *
 * @tparam T floating point type
 * @tparam Iterator Iterator type
 * @tparam Collection Collection type
 *
 * @param collection collection of primitives
 * @param begin iterator specifying the beginning of the primitives from the collection to insert in the leaf
 * @param end iterator specifying the end of the primitives from the collection to insert in the leaf
 * @param bmin vertex describing the minimum extent of the bounding box
 * @param bmax vertex describing the maximum extent of the bounding box
 * @param statistics BVHBuildStatistics structure to keep track of the build
 * @param options Build options
 * @return std::pair<BVHNode<T, Collection>, Iterator> with the newly created branch node as well as the iterator to the
 * middle of the node list.
 */
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

/**
 * This function finds the axis and the iterator position for which the next volume split needs to be done.
 *
 * @tparam T floating point type
 * @tparam Iterator Iterator type
 * @tparam Collection Collection type
 *
 * @param collection collection of primitives
 * @param begin iterator specifying the beginning of the primitives from the collection to insert in the leaf
 * @param end iterator specifying the end of the primitives from the collection to insert in the leaf
 * @param bmin vertex describing the minimum extent of the bounding box
 * @param bmax vertex describing the maximum extent of the bounding box
 * @param statistics BVHBuildStatistics structure to keep track of the build
 * @param options Build options
 * @return std::pair<unsigned int, Iterator> describing the cut_axis and the mid position for the cut
 */
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
