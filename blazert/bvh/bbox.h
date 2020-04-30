#pragma once
#ifndef BLAZERT_BVH_BBOX_H_
#define BLAZERT_BVH_BBOX_H_

#include <iostream>
#include <thread>

#include <blazert/datatypes.h>
#include <blazert/defines.h>

namespace blazert {

/**
 * @brief Bounding box.
 */
template<typename T>
class alignas(sizeof(Vec3r<T>)) BBox {
public:
  Vec3r<T> bmin;
  Vec3r<T> bmax;

  BBox() {
    bmin = std::numeric_limits<T>::max();
    bmax = -std::numeric_limits<T>::max();
  }
};

template<typename T>
std::ostream& operator<<(std::ostream& stream, const BBox<T>& b) {
  stream << "BoundingBox" << std::endl;
  stream << " - Lower:" << std::endl;
  stream << b.bmin << std::endl;
  stream << " - Upper:" << std::endl;
  stream << b.bmax << std::endl;
  return stream;
}


template<typename T>
inline void GetBoundingBox(Vec3r<T> &bmin, Vec3r<T> &bmax, const std::vector<BBox<T>> &bboxes,
                           const std::vector<unsigned int> &indices, unsigned int left_index, unsigned int right_index) {

  unsigned int i = left_index;
  unsigned int idx = indices[i];

  bmin = bboxes[idx].bmin;
  bmax = bboxes[idx].bmax;

  // for each primitive
  for (i = left_index + 1; i < right_index; i++) {
    idx = indices[i];
    // xyz
    for (int k = 0; k < 3; k++) {
      bmin[k] = std::min(bmin[k], bboxes[idx].bmin[k]);
      bmax[k] = std::max(bmax[k], bboxes[idx].bmax[k]);
    }
  }
}

template<typename T, class P>
inline void ComputeBoundingBox(Vec3r<T> &bmin, Vec3r<T> &bmax, std::vector<unsigned int> &indices,
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
