#pragma once
#ifndef BLAZERT_BVH_BINBUFFER_H_
#define BLAZERT_BVH_BINBUFFER_H_

namespace blazert {

template<typename T> inline T calculate_box_surface(const Vec3r<T> &min, const Vec3r<T> &max) {
  const Vec3r<T> box{blaze::abs(max - min)};
  return static_cast<T>(2.0) * (box[0] * box[1] + box[1] * box[2] + box[2] * box[0]);
}

template <typename T>
struct BLAZERTALIGN Bin {
  Vec3r<T> min;
  Vec3r<T> max;
  unsigned int count;
  T cost;

  Bin() : min(std::numeric_limits<T>::max()), max(-std::numeric_limits<T>::max()), count(0), cost(static_cast<T>(0.)) {}
};

template<class T>
struct BLAZERTALIGN BinBuffer {
  explicit BinBuffer(const unsigned int size) : size(size) {
    bin.resize(3*size);  // For each axis.
  }

  void clear() {
    bin.clear();
    bin.resize(3*size);
  }

  std::vector<Bin<T>> bin;
  const unsigned int size;
};

template<typename T, class Collection>
inline void contribute_bins(BinBuffer<T> &bins, const Vec3r<T> &scene_min, const Vec3r<T> &scene_max,
                                const std::vector<unsigned int> &indices, const unsigned int left_idx,
                                const unsigned int right_idx, const Collection &p) {

  const Vec3r<T> scene_size{blaze::abs(scene_max - scene_min)};
  Vec3r<T> scene_inv_size;

  for (unsigned int i = 0; i < 3; i++)
    scene_inv_size[i] = (scene_size[i] > static_cast<T>(0.)) ? bins.size / scene_size[i] : static_cast<T>(0.);

  bins.clear();

  Vec3r<T> bmin, bmax, center;

  for (unsigned int i=left_idx; i<right_idx; i++) {

    p.get_primitive_bounding_box(bmin, bmax, indices[i]);
    p.get_primitive_center(center, indices[i]);

    // Quantize the center position into [0, BIN_SIZE)
    // TODO: This fails if size in any axis =0.
    const Vec3r<T> quantized_center{(center - scene_min) * scene_inv_size};  // TODO: in [0., T(bin_size)]

    for (unsigned int j = 0; j < 3; j++) {
      unsigned int idx = std::min(bins.size-1, unsigned(std::max(0, int(quantized_center[j]))));

      // Increment bin counter + extend bounding box of bin
      Bin<T> &bin = bins.bin[j * bins.size + idx];
      bin.count++;

      for (unsigned int k = 0; k < 3; k++) {
        bin.min[k] = std::min(bin.min[k], bmin[k]);
        bin.max[k] = std::max(bin.max[k], bmax[k]);
      }
    }
  }
}

template<typename T>
inline unsigned int find_cut_from_bins(Vec3r<T> &cut_pos, BinBuffer<T> &bins, const Vec3r<T> &bmin, const Vec3r<T> &bmax) {

  T min_cost[3];

  for (unsigned int j = 0; j < 3; j++) {
    min_cost[j] = std::numeric_limits<T>::max();

    // Sweep left to accumulate bounding boxes and compute the right-hand side of the cost
    unsigned int count = 0;
    Vec3r<T> bmin_{std::numeric_limits<T>::max()};
    Vec3r<T> bmax_{-std::numeric_limits<T>::max()};

    for (unsigned int i = bins.size - 1; i > 0; i--) {
      Bin<T>& bin = bins.bin[j * bins.size + i];
      for (unsigned int k = 0; k < 3; k++) {
        bmin_[k] = std::min(bin.min[k], bmin_[k]);
        bmax_[k] = std::max(bin.max[k], bmax_[k]);
      }
      count += bin.count;
      bin.cost = count * calculate_box_surface(bmin_, bmax_);
    }

    // Sweep right to compute the full cost
    count = 0;
    bmin_ = std::numeric_limits<T>::max();
    bmax_ = -std::numeric_limits<T>::max();

    unsigned int min_bin = 1;

    for (unsigned int i = 0; i < bins.size - 1; i++) {
      Bin<T>& bin = bins.bin[j * bins.size + i];
      Bin<T>& next_bin = bins.bin[j * bins.size + i + 1];
      for (int k = 0; k < 3; ++k) {
        bmin_[k] = std::min(bin.min[k], bmin_[k]);
        bmax_[k] = std::max(bin.max[k], bmax_[k]);
      }
      count += bin.count;
      // Traversal cost and intersection cost are irrelevant for minimization
      T cost = count * calculate_box_surface(bmin_, bmax_) + next_bin.cost;

      if (cost < min_cost[j]) {
        min_cost[j] = cost;
        // Store the beginning of the right partition
        min_bin = i + 1;
      }
    }
    cut_pos[j] = min_bin * ((bmax[j] - bmin[j]) / bins.size) + bmin[j];
  }

  unsigned int min_cost_axis = 0;
  if (min_cost[0] > min_cost[1]) min_cost_axis = 1;
  if (min_cost[min_cost_axis] > min_cost[2]) min_cost_axis = 2;

  return min_cost_axis;
}

}// namespace blazert

#endif// BLAZERT_BVH_BINBUFFER_H_
