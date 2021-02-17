#pragma once
#ifndef BLAZERT_BVH_BBOX_H_
#define BLAZERT_BVH_BBOX_H_

#include <limits>
#include <utility>

#include <blazert/datatypes.h>
#include <blazert/defines.h>

namespace blazert {

/**
 * This function computes the common bounding box of multiple primitives specified by the range of primitives
 * [first; last] in the primitive collection p.
 *
 * @tparam T floating point type
 * @tparam Iterator iterator type
 * @tparam Collection primitive collection.
 *
 * @param p collection of primitives
 * @param first iterator specifying the first element to consider for bounding box computation
 * @param last iterator specifying the last element to consider for bounding box computation
 *
 * @return std::pair<blazert::Vec3r<T>, blazert::Vec3r<T>> which holds the minimum and maximum vertex spanning the bounding box
 */
template<typename T, typename Iterator, class Collection>
inline std::pair<Vec3r<T>, Vec3r<T>> compute_bounding_box(const Collection &p, Iterator first, Iterator last) {

  Vec3r<T> min(std::numeric_limits<T>::max());
  Vec3r<T> max(-std::numeric_limits<T>::max());

  for (auto it = first; it != last; ++it) {
    const auto [min_, max_] = p.get_primitive_bounding_box(*it);
    unity(min, max, min_, max_);
  }

  return std::make_pair(std::move(min), std::move(max));
}
}// namespace blazert

#endif// BLAZERT_BVH_BBOX_H_