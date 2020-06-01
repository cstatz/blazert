#pragma once
#ifndef BLAZERT_BVH_OPTIONS_H_
#define BLAZERT_BVH_OPTIONS_H_

#include <blazert/defines.h>
#include <blazert/datatypes.h>

namespace blazert {

/**
 * @brief BVH Build Options
 * @tparam T real value (e.g. float, double ...), gets passed into blaze vector types.
 */
template<typename T>
struct BLAZERTALIGN BVHBuildOptions {
public:
  unsigned int min_leaf_primitives;
  unsigned int max_tree_depth;
  unsigned int bin_size;

  BVHBuildOptions()
      :
        min_leaf_primitives(4),
        max_tree_depth(BLAZERT_MAX_TREE_DEPTH),  // Does not impact build time significantly
        bin_size(64)  // Influences build significantly
        {}
};

/**
 * @brief BVH trace options.
 */
template<typename T>
class BLAZERTALIGN BVHTraceOptions {
public:
  bool cull_back_face;

  BVHTraceOptions() : cull_back_face(false) {}
};
}// namespace blazert

#endif// BLAZERT_BVH_OPTIONS_H_
