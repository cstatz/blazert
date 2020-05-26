#pragma once
#ifndef BLAZERT_BVH_BINBUFFER_H_
#define BLAZERT_BVH_BINBUFFER_H_

namespace blazert {

template<typename T> inline T calculate_box_surface(const Vec3r<T> &min, const Vec3r<T> &max) {
  const Vec3r<T> box = max - min;
  return static_cast<T>(2.0) * (box[0] * box[1] + box[1] * box[2] + box[2] * box[0]);
}

template <typename T>
struct BLAZERTALIGN Bin {
  Vec3r<T> min;
  Vec3r<T> max;
  size_t  count;
  T cost;

  Bin() : count(0), cost(0) {}
};

template<class T>
struct BLAZERTALIGN BinBuffer {
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
inline void ContributeBinBuffer(BinBuffer<T> &bins, const Vec3r<T> &scene_min, const Vec3r<T> &scene_max, const std::vector<unsigned int> &indices,
                                const unsigned int left_idx, const unsigned int right_idx, const P &p) {

  T bin_size = static_cast<T>(bins.bin_size);

  // Calculate extent
  Vec3r<T> scene_size, scene_inv_size;
  scene_size = scene_max - scene_min;

  for (int i = 0; i < 3; ++i) {

    if (scene_size[i] > static_cast<T>(0.0)) {
      scene_inv_size[i] = bin_size / scene_size[i];
    }
    else {
      scene_inv_size[i] = static_cast<T>(0.0);
    }
  }

  // Clear bin data
  bins.clear();

  for (unsigned int i=left_idx; i<right_idx; i++) {
    //
    // Quantize the center position into [0, BIN_SIZE)
    //
    // q[i] = (int)(p[i] - scene_bmin) / scene_size
    //
    Vec3r<T> bmin, bmax, center;

    p.BoundingBoxAndCenter(bmin, bmax, center, indices[i]);
    Vec3r<T> quantized_center = (center - scene_min) * scene_inv_size;  // in [0., T(bin_size)]

    // idx is now in [0, BIN_SIZE)
    for (int j = 0; j < 3; ++j) {
      // idx is now in [0, BIN_SIZE)
      unsigned int idx = std::min(bins.bin_size - 1, unsigned(std::max(0, int(quantized_center[j]))));

      // Increment bin counter + extend bounding box of bin
      Bin<T>& bin = bins.bin[j * bins.bin_size + idx];
      bin.count++;
      for (int k = 0; k < 3; ++k) {
        bin.min[k] = std::min(bin.min[k], bmin[k]);
        bin.max[k] = std::max(bin.max[k], bmax[k]);
      }
    }
  }
}

template<typename T>
inline unsigned int FindCutFromBinBuffer(Vec3r<T> &cut_pos, BinBuffer<T> &bins, const Vec3r<T> &bmin, const Vec3r<T> &bmax) {// should be in [0.0, 1.0]

  int minCostAxis;
  T minCost[3];

  for (int j = 0; j < 3; ++j) {
    minCost[j] = std::numeric_limits<T>::max();

    // Sweep left to accumulate bounding boxes and compute the right-hand side of the cost
    size_t count = 0;
    Vec3r<T> bmin_, bmax_;

    for (size_t i = bins.bin_size - 1; i > 0; --i) {
      Bin<T>& bin = bins.bin[j * bins.bin_size + i];
      for (int k = 0; k < 3; ++k) {
        bmin_[k] = std::min(bin.min[k], bmin_[k]);
        bmax_[k] = std::max(bin.max[k], bmax_[k]);
      }
      count += bin.count;
      bin.cost = count * calculate_box_surface(bmin_, bmax_);
    }

    // Sweep right to compute the full cost
    count = 0;
    bmin_ = static_cast<T>(0.);
    bmax_ = static_cast<T>(0.);

    size_t minBin = 1;
    for (size_t i = 0; i < bins.bin_size - 1; i++) {
      Bin<T>& bin = bins.bin[j * bins.bin_size + i];
      Bin<T>& next_bin = bins.bin[j * bins.bin_size + i + 1];
      for (int k = 0; k < 3; ++k) {
        bmin_[k] = std::min(bin.min[k], bmin_[k]);
        bmax_[k] = std::max(bin.max[k], bmax_[k]);
      }
      count += bin.count;
      // Traversal cost and intersection cost are irrelevant for minimization
      T cost = count * calculate_box_surface(bmin_, bmax_) + next_bin.cost;

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
