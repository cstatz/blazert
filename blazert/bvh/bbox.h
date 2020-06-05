#pragma once
#ifndef BLAZERT_BVH_BBOX_H_
#define BLAZERT_BVH_BBOX_H_

#include <blazert/datatypes.h>
#include <blazert/defines.h>

namespace blazert {

template<typename T, class Collection>
inline void compute_bounding_box(Vec3r<T> &min, Vec3r<T> &max, const std::vector<unsigned int> &indices,
                               const unsigned int left_index, const unsigned int right_index, const Collection &p) {

  Vec3r<T> _min, _max;

  for (unsigned int i = left_index; i < right_index; i++) {
    p.get_primitive_bounding_box(_min, _max, indices[i]);

    for (int k = 0; k < 3; k++) {
      min[k] = std::min(min[k], _min[k]);
      max[k] = std::max(max[k], _max[k]);
    }
  }
}

}// namespace blazert

#endif// BLAZERT_BVH_BBOX_H_
