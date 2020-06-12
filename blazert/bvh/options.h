#pragma once
#ifndef BLAZERT_BVH_OPTIONS_H_
#define BLAZERT_BVH_OPTIONS_H_

#include <blazert/datatypes.h>
#include <blazert/defines.h>

namespace blazert {

/**
 * @brief BVH Build Options
 * @tparam T real value (e.g. float, double ...), gets passed into blaze vector types.
 */
template<typename T>
struct BLAZERTALIGN BVHBuildOptions {
public:
  const unsigned int min_leaf_primitives;
  const unsigned int max_tree_depth;
  const unsigned int bin_size;

  explicit BVHBuildOptions(const unsigned int min_leaf_primitives = 4,
                           const unsigned int max_tree_depth = BLAZERT_MAX_TREE_DEPTH, const unsigned int bin_size = 64)
      : min_leaf_primitives(min_leaf_primitives),
        max_tree_depth(
            max_tree_depth),/// Does not impact build time significantly, but impacts memory demand and traversal time.
        bin_size(bin_size)  /// Influences build (-time) significantly. Has little impact on traversal time.
  {}
};

}// namespace blazert

#endif// BLAZERT_BVH_OPTIONS_H_
