#pragma once
#ifndef BLAZERT_BVH_OPTIONS_H_
#define BLAZERT_BVH_OPTIONS_H_

#include <blazert/defines.h>

namespace blazert {

/// BVH build option.
template<typename T>
struct BVHBuildOptions {
  T cost_t_aabb;
  unsigned int min_leaf_primitives;
  unsigned int max_tree_depth;
  unsigned int bin_size;
  unsigned int shallow_depth;
  unsigned int min_primitives_for_parallel_build;

  // Cache bounding box computation.
  // Requires more memory, but BVHbuild can be faster.
  bool cache_bbox;
  unsigned char pad[3];

  // Set default value: Taabb = 0.2
  BVHBuildOptions()
      : cost_t_aabb(static_cast<T>(0.2)),
        min_leaf_primitives(4),
        max_tree_depth(256),
        bin_size(64),
        shallow_depth(kBLAZERT_SHALLOW_DEPTH),
        min_primitives_for_parallel_build(
            kBLAZERT_MIN_PRIMITIVES_FOR_PARALLEL_BUILD),
        cache_bbox(false) {}
};

///
/// @brief BVH trace option.
///
class BVHTraceOptions {
public:
  // Hit only for face IDs in indexRange.
  // This feature is good to mimic something like glDrawArrays()
  unsigned int prim_ids_range[2];

  // Prim ID to skip for avoiding self-intersection
  // -1 = no skipping
  unsigned int skip_prim_id;

  bool cull_back_face;
  unsigned char pad[3];///< Padding (not used)

  BVHTraceOptions() {
    prim_ids_range[0] = 0;
    prim_ids_range[1] = 0x7FFFFFFF;// Up to 2G face IDs.

    skip_prim_id = static_cast<unsigned int>(-1);
    cull_back_face = false;
  }
};
}// namespace blazert

#endif// BLAZERT_BVH_OPTIONS_H_
