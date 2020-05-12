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
struct BLAZEALIGN BVHBuildOptions {
public:
  T cost_t_aabb;
  bool cache_bbox;
  unsigned int min_leaf_primitives;
  unsigned int max_tree_depth;
  unsigned int bin_size;
#ifdef BLAZERT_PARALLEL_BUILD
  unsigned int shallow_depth;
  unsigned int min_primitives_for_parallel_build;
#endif

  // Set default value: T cost_t_aabb = 0.2
  BVHBuildOptions()
      : cost_t_aabb(static_cast<T>(0.2)),
        cache_bbox(false),
        min_leaf_primitives(4),
        max_tree_depth(256),
        bin_size(64)
#ifdef BLAZERT_PARALLEL_BUILD
        ,
        shallow_depth(BLAZERT_SHALLOW_DEPTH),
        min_primitives_for_parallel_build(BLAZERT_MIN_PRIMITIVES_FOR_PARALLEL_BUILD),
#endif
        {}
};

/**
 * @brief BVH trace options.
 */
template<typename T>
class BLAZEALIGN BVHTraceOptions {
public:
  // TODO: is this really necessary?
  // Hit only for face IDs in indexRange.
  // This feature is good to mimic something like glDrawArrays()
  vec2ui prim_ids_range;

  // Prim ID to skip for avoiding self-intersection
  // -1 = no skipping
  unsigned int skip_prim_id;
  bool cull_back_face;

  BVHTraceOptions() : prim_ids_range({0, 0x7FFFFFFF}), skip_prim_id(-1), cull_back_face(false) {}  // 0x7FFFFFFF -> 2e9 prims.
};
}// namespace blazert

#endif// BLAZERT_BVH_OPTIONS_H_
