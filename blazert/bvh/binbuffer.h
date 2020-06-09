#pragma once
#ifndef BLAZERT_BVH_BINBUFFER_H_
#define BLAZERT_BVH_BINBUFFER_H_

namespace blazert {

template<typename T>
inline T calculate_box_surface(const Vec3r<T> &min, const Vec3r<T> &max) {
  const Vec3r<T> box{blaze::abs(max - min)};
  return static_cast<T>(2.0) * (box[0] * box[1] + box[1] * box[2] + box[2] * box[0]);
}

template<typename T>
struct BLAZERTALIGN Bin {
  Vec3r<T> min;
  Vec3r<T> max;
  unsigned int count;

  Bin() : min(std::numeric_limits<T>::max()), max(-std::numeric_limits<T>::max()), count(0) {}
};

template<class T>
struct BLAZERTALIGN BinBuffer {
  explicit BinBuffer(const unsigned int size) : size(size) {
    bin.resize(3 * size);// For each axis.
  }

  void clear() {
    bin.clear();
    bin.resize(3 * size);
  }

  std::vector<Bin<T>> bin;
  const unsigned int size;
};

template<typename T, typename Iterator, class Collection, typename Options>
inline BinBuffer<T> sort_collection_into_bins(const Collection &p, Iterator begin, Iterator end,
                                              const Vec3r<T> &min, const Vec3r<T> &max, const Options &options) {

  BinBuffer<T> bins(options.bin_size);
  const Vec3r<T> size{max - min};
  Vec3r<T> inv_size;

  for (unsigned int i = 0; i < 3; i++)
    inv_size[i] = (size[i] > static_cast<T>(0.)) ? 1. / size[i] : static_cast<T>(0.);

  for (auto it = begin; it != end; ++it) {

    auto [bmin, bmax] = p.get_primitive_bounding_box(*it);
    const auto center = p.get_primitive_center(*it);

    // assert center > min
    const Vec3ui normalized_center{(center - min) * inv_size * (bins.size - 1)};// 0 .. 63

    for (unsigned int j = 0; j < 3; j++) {

      if (inv_size[j] > static_cast<T>(0.)) {
        unsigned int idx = std::min(bins.size - 1, unsigned(std::max(static_cast<unsigned int>(0), normalized_center[j])));

        Bin<T> &bin = bins.bin[j * bins.size + idx];
        bin.count++;
        unity(bin.min, bin.max, bmin, bmax);
      }
    }
  }

  return bins;
}

template<typename T, typename Iterator, template<typename> typename Collection, typename Options>
inline std::pair<unsigned int, Vec3r<T>> find_best_split_binned(const Collection<T> &collection,
                                                                Iterator begin, Iterator end,
                                                                const Vec3r<T> &min, const Vec3r<T> &max, const Options &options) {

  auto bins = sort_collection_into_bins(collection, begin, end, min, max, options);

  Vec3r<T> cut_pos;
  Vec3r<T> min_cost{std::numeric_limits<T>::max()};

  std::vector<T> left_cost, right_cost;
  left_cost.resize(options.bin_size);
  right_cost.resize(options.bin_size);

  // iterating over all 3 axes
  for (unsigned int j = 0; j < 3; j++) {

    // Sweep left to accumulate bounding boxes and compute the right-hand side of the cost
    unsigned int count = 0;
    Vec3r<T> min_{std::numeric_limits<T>::max()};
    Vec3r<T> max_{-std::numeric_limits<T>::max()};

    for (unsigned int i = bins.size - 1; i > 0; i--) {
      Bin<T> &bin = bins.bin[j * bins.size + i];
      unity(min_, max_, bin.min, bin.max);
      count += bin.count;
      left_cost[i] = count * calculate_box_surface(min_, max_);
    }

    // Sweep right to compute the full cost
    count = 0;
    min_ = std::numeric_limits<T>::max();
    max_ = -std::numeric_limits<T>::max();

    for (unsigned int i = 0; i < bins.size; i++) {
      Bin<T> &bin = bins.bin[j * bins.size + i];
      unity(min_, max_, bin.min, bin.max);
      count += bin.count;
      right_cost[i] = count * calculate_box_surface(min_, max_);
    }

    // Store the beginning of the correct partition
    for (unsigned int i = 1; i < bins.size; i++) {
      if (right_cost[i - 1] < left_cost[i]) {
        min_cost[j] = right_cost[i - 1];
        cut_pos[j] = i * ((max[j] - min[j]) / bins.size) + min[j];
        break;
      }
    }
  }

  unsigned int min_cost_axis = 0;
  if (min_cost[0] > min_cost[1])
    min_cost_axis = 1;
  if (min_cost[min_cost_axis] > min_cost[2])
    min_cost_axis = 2;

  return std::make_pair(min_cost_axis, cut_pos);
}

}// namespace blazert

#endif// BLAZERT_BVH_BINBUFFER_H_
