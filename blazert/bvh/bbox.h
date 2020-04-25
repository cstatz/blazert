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
                               unsigned int left_index,
                               unsigned int right_index, const P &p) {
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

//#ifdef _OPENMP
//template<typename T, class P>
//void ComputeBoundingBoxOMP(Vec3r<T> &bmin, Vec3r<T> &bmax, std::vector<unsigned int> &indices, unsigned int left_index,
//                           unsigned int right_index, const P &p) {
//  { p.BoundingBox(bmin, bmax, indices[left_index]); }
//
//  Vec3r<T> local_bmin = bmin;
//  Vec3r<T> local_bmax = bmax;
//  unsigned int n = right_index - left_index;
//
//#pragma omp parallel firstprivate(local_bmin, local_bmax) if (n > (1024 * 128))
//  {
//#pragma omp parallel for
//    // for each face
//    for (int i = int(left_index); i < int(right_index); i++) {
//      unsigned int idx = indices[i];
//
//      Vec3r<T> bbox_min, bbox_max;
//      p.BoundingBox(bbox_min, bbox_max, idx);
//
//      // xyz
//      for (int k = 0; k < 3; k++) {
//        bmin[k] = std::min(bmin[k], bbox_min[k]);
//        bmax[k] = std::max(bmax[k], bbox_max[k]);
//      }
//    }
//
//#pragma omp critical
//    {
//      for (int k = 0; k < 3; k++) {
//        bmin[k] = std::min(bmin[k], local_bmin[k]);
//        bmax[k] = std::max(bmax[k], local_bmax[k]);
//      }
//    }
//  }
//}
//#endif
//
//#if __cplusplus >= 201103L
//template<typename T, class P>
//inline void ComputeBoundingBoxThreaded(Vec3r<T> &bmin, Vec3r<T> &bmax, std::vector<unsigned int> &indices,
//                                       unsigned int left_index,
//                                       unsigned int right_index, const P &p) {
//  unsigned int n = right_index - left_index;
//
//  size_t num_threads = std::min(
//      size_t(kBLAZERT_MAX_THREADS),
//      std::max(size_t(1), size_t(std::thread::hardware_concurrency())));
//
//  if (n < num_threads) {
//    num_threads = n;
//  }
//
//  std::vector<std::thread> workers;
//
//  size_t ndiv = n / num_threads;
//
//  std::vector<Vec3r<T>> local_bmins;
//  std::vector<Vec3r<T>> local_bmaxs;
//
//  local_bmins.resize(num_threads);
//  local_bmaxs.resize(num_threads);
//
//  for (size_t t = 0; t < num_threads; t++) {
//    workers.emplace_back(std::thread([&, t]() {
//      size_t si = left_index + t * ndiv;
//      size_t ei = (t == (num_threads - 1)) ? size_t(right_index) : std::min(left_index + (t + 1) * ndiv, size_t(right_index));
//
//      local_bmins[t] = std::numeric_limits<T>::infinity();
//      local_bmaxs[t] = -std::numeric_limits<T>::infinity();
//
//      // for each face
//      for (size_t i = si; i < ei; i++) {
//        unsigned int idx = indices[i];
//
//        Vec3r<T> bbox_min, bbox_max;
//        p.BoundingBox(bbox_min, bbox_max, idx);
//
//        // xyz
//        for (size_t k = 0; k < 3; k++) {
//          local_bmins[t][k] = std::min(local_bmins[t][k], bbox_min[int(k)]);
//          local_bmaxs[t][k] = std::max(local_bmaxs[t][k], bbox_max[int(k)]);
//        }
//      }
//    }));
//  }
//
//  for (auto &t : workers) {
//    t.join();
//  }
//
//  // merge bbox
//  bmin = local_bmins[0];
//  bmax = local_bmaxs[0];
//
//  for (size_t t = 1; t < num_threads; t++) {
//    for (size_t k = 0; k < 3; k++) {
//      bmin[int(k)] = std::min(bmin[int(k)], local_bmins[t][k]);
//      bmax[int(k)] = std::max(bmax[int(k)], local_bmaxs[t][k]);
//    }
//  }
//}
//#endif

}// namespace blazert

#endif// BLAZERT_BVH_BBOX_H_
