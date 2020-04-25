#pragma once
#ifndef BLAZERT_BVH_BINBUFFER_H_
#define BLAZERT_BVH_BINBUFFER_H_

#include <blazert/bvh/sah.h>
#include <cstring>
#include <iostream>

namespace blazert {

template<typename T>
inline T CalculateSurfaceArea(const Vec3r<T> &min, const Vec3r<T> &max) {
  Vec3r<T> box = max - min;
  return static_cast<T>(2.0) * (box[0] * box[1] + box[1] * box[2] + box[2] * box[0]);
}

template <typename T>
struct alignas(Vec3r<T>) Bin {
  BBox<T> bbox;
  size_t  count;
  T cost;

  Bin() : count(0), cost(0) {}
};

template<class T>
struct alignas(Vec3r<T>) BinBuffer {
  explicit BinBuffer(unsigned int size) {
    bin_size = size;
    // TODO: This is not obvious ..
    bin.resize(3 * size);
    clear();
  }

  void clear() {
    std::fill(bin.begin(), bin.end(), Bin<T>());
  }

  std::vector<Bin<T>> bin;
  unsigned int bin_size;
};

template<typename T, class P>
inline void ContributeBinBuffer(BinBuffer<T> &bins,// [out]
                                const Vec3r<T> &scene_min,
                                const Vec3r<T> &scene_max,
                                std::vector<unsigned int> &indices, unsigned int left_idx,
                                unsigned int right_idx, const P &p) {

  T bin_size = static_cast<T>(bins.bin_size);

  // Calculate extent
  Vec3r<T> scene_size, scene_inv_size;
  scene_size = scene_max - scene_min;

  for (int i = 0; i < 3; ++i) {
    assert(scene_size[i] >= static_cast<T>(0.0));

    if (scene_size[i] > static_cast<T>(0.0)) {
      scene_inv_size[i] = bin_size / scene_size[i];
    }
    else {
      scene_inv_size[i] = static_cast<T>(0.0);
    }
  }

  // Clear bin data
  bins.clear();

  for (size_t i = left_idx; i < right_idx; i++) {
    //
    // Quantize the center position into [0, BIN_SIZE)
    //
    // q[i] = (int)(p[i] - scene_bmin) / scene_size
    //
    Vec3r<T> bmin, bmax, center;

    p.BoundingBoxAndCenter(bmin, bmax, center, indices[i]);
    // GetBoundingBoxOfTriangle(&bmin, &bmax, vertices, faces, indices[i]);

    //Vec3r<T> quantized_bmin = (bmin - scene_min) * scene_inv_size;
    //Vec3r<T> quantized_bmax = (bmax - scene_min) * scene_inv_size;
    Vec3r<T> quantized_center = (center - scene_min) * scene_inv_size;

    // idx is now in [0, BIN_SIZE)
    for (int j = 0; j < 3; ++j) {
      // idx is now in [0, BIN_SIZE)
      unsigned idx = std::min(bins.bin_size - 1, unsigned(std::max(0, int(quantized_center[j]))));

      // Increment bin counter + extend bounding box of bin
      Bin<T>& bin = bins.bin[j * bins.bin_size + idx];
      bin.count++;
      for (int k = 0; k < 3; ++k) {
        bin.bbox.bmin[k] = std::min(bin.bbox.bmin[k], bmin[k]);
        bin.bbox.bmax[k] = std::max(bin.bbox.bmax[k], bmax[k]);
      }
    }
  }
}

template<typename T>
inline unsigned int FindCutFromBinBuffer(Vec3r<T> &cut_pos,// [out] xyz
                                         BinBuffer<T> &bins, const Vec3r<T> &bmin,
                                         const Vec3r<T> &bmax, size_t num_primitives) {// should be in [0.0, 1.0]
  int minCostAxis;
  T minCost[3];

  for (int j = 0; j < 3; ++j) {
    minCost[j] = std::numeric_limits<T>::max();

    // Sweep left to accumulate bounding boxes and compute the right-hand side of the cost
    size_t count = 0;
    BBox<T> accumulated_bbox;
    for (size_t i = bins.bin_size - 1; i > 0; --i) {
      Bin<T>& bin = bins.bin[j * bins.bin_size + i];
      for (int k = 0; k < 3; ++k) {
        accumulated_bbox.bmin[k] = std::min(bin.bbox.bmin[k], accumulated_bbox.bmin[k]);
        accumulated_bbox.bmax[k] = std::max(bin.bbox.bmax[k], accumulated_bbox.bmax[k]);
      }
      count += bin.count;
      bin.cost = count * CalculateSurfaceArea(accumulated_bbox.bmin, accumulated_bbox.bmax);
    }

    // Sweep right to compute the full cost
    count = 0;
    accumulated_bbox = BBox<T>();
    size_t minBin = 1;
    for (size_t i = 0; i < bins.bin_size - 1; i++) {
      Bin<T>& bin = bins.bin[j * bins.bin_size + i];
      Bin<T>& next_bin = bins.bin[j * bins.bin_size + i + 1];
      for (int k = 0; k < 3; ++k) {
        accumulated_bbox.bmin[k] = std::min(bin.bbox.bmin[k], accumulated_bbox.bmin[k]);
        accumulated_bbox.bmax[k] = std::max(bin.bbox.bmax[k], accumulated_bbox.bmax[k]);
      }
      count += bin.count;
      // Traversal cost and intersection cost are irrelevant for minimization
      T cost = count * CalculateSurfaceArea(accumulated_bbox.bmin, accumulated_bbox.bmax) + next_bin.cost;

      if (cost < minCost[j]) {
        minCost[j] = cost;
        // Store the beginning of the right partition
        minBin = i + 1;
      }
    }
    cut_pos[j] = minBin * ((bmax[j] - bmin[j]) / bins.bin_size) + bmin[j];
  }

  minCostAxis = 0;
  if (minCost[0] > minCost[1]) minCostAxis = 1;
  if (minCost[minCostAxis] > minCost[2]) minCostAxis = 2;

  return minCostAxis;
}

}// namespace blazert

#endif// BLAZERT_BVH_BINBUFFER_H_
