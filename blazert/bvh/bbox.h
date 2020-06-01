#pragma once
#ifndef BLAZERT_BVH_BBOX_H_
#define BLAZERT_BVH_BBOX_H_

#include <iostream>
#include <thread>

#include <blazert/datatypes.h>
#include <blazert/defines.h>

namespace blazert {

template<typename T, class P>
inline void compute_bounding_box(Vec3r<T> &bmin, Vec3r<T> &bmax, std::vector<unsigned int> &indices,
                               unsigned int left_index, unsigned int right_index, const P &p) {
  unsigned int idx = indices[left_index];
  p.BoundingBox(bmin, bmax, idx);

  {
    // for each primitive
    for (unsigned int i = left_index + 1; i < right_index; i++) {
      idx = indices[i];
      Vec3r<T> bbox_min, bbox_max;
      p.BoundingBox(bbox_min, bbox_max, idx);

      // xyz
      for (int k = 0; k < 3; k++) {
        bmin[k] = std::min(bmin[k], bbox_min[k]);
        bmax[k] = std::max(bmax[k], bbox_max[k]);
      }
    }
  }
}

}// namespace blazert

#endif// BLAZERT_BVH_BBOX_H_
