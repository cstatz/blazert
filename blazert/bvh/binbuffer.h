#pragma once
#ifndef BLAZERT_BVH_BINBUFFER_H_
#define BLAZERT_BVH_BINBUFFER_H_

namespace blazert {

/**
 * This function calculates the surface area of the box specified by the two vertices min and max.
 *
 * @tparam T floating point type
 * @param min one vertex of the box
 * @param max vertex diagonally opposite from the min vertex
 * @return surface area of the box
 */
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
  T cost;

  Bin() : min(std::numeric_limits<T>::max()), max(-std::numeric_limits<T>::max()), count(0), cost(static_cast<T>(0)){};
  Bin(Bin &&rhs) noexcept
      : min(std::move(rhs.min)), max(std::move(rhs.max)), count(std::exchange(rhs.count, 0)),
        cost(std::exchange(rhs.cost, static_cast<T>(0.))){};
};

template<class T>
struct BLAZERTALIGN BinBuffer {
  explicit BinBuffer(const unsigned int size) : size(size) {
    bin.resize(3 * size);// For each axis.
  }

  BinBuffer(BinBuffer &&rhs) noexcept : bin(std::move(rhs.bin)), size(std::exchange(rhs.size, static_cast<T>(0.))){};

  void clear() {
    bin.clear();
    bin.resize(3 * size);
  }

  std::vector<Bin<T>> bin;
  unsigned int size;
};

/**
 * This function sorts the primitives contained in the collection into bins alongside each axis
 *
 * @tparam T floating point type
 * @tparam Iterator Iterator type
 * @tparam Collection Collection type
 * @tparam Options build options type
 * @param p collection of primitives
 * @param begin iterator to the beginning of the relevant primitives in the collection
 * @param end iterator to the end of the relevant primitives in the collection
 * @param min vertex to the mimimum extent of the boundgin box under consideration
 * @param max vertex to the maximum extent of the bounding box under consideration
 * @param options build options
 * @return BinBuffer containing the bins
 */
template<typename T, typename Iterator, class Collection, typename Options>
inline BinBuffer<T> sort_collection_into_bins(const Collection &p, Iterator begin, Iterator end, const Vec3r<T> &min,
                                              const Vec3r<T> &max, const Options &options) {

  BinBuffer<T> bins(options.bin_size);
  const Vec3r<T> size{max - min};
  Vec3r<T> inv_size;

  for (unsigned int i = 0; i < 3; i++)
    inv_size[i] = (size[i] > static_cast<T>(0.)) ? static_cast<T>(1.) / size[i] : static_cast<T>(0.);

  for (auto it = begin; it != end; ++it) {

    const auto [bmin, bmax] = p.get_primitive_bounding_box(*it);
    const auto center = p.get_primitive_center(*it);

    // assert center > min
    const Vec3r<T> normalized_center{(center - min) * inv_size * (bins.size - 1)};// 0 .. 63

    for (unsigned int j = 0; j < 3; j++) {
      unsigned int idx = std::min(
          bins.size - 1, unsigned(std::max(static_cast<unsigned int>(0), unsigned(std::round(normalized_center[j])))));
      Bin<T> &bin = bins.bin[j * bins.size + idx];
      bin.count++;
      unity(bin.min, bin.max, bmin, bmax);
    }
  }

  return bins;
}

/**
 * This function finds the best split along one axis for primitives in the collection within the interval [begin; end]
 *
 * @tparam T floating point type
 * @tparam Iterator Iterator type
 * @tparam Collection Collection type
 * @tparam Options Build Options Type
 *
 * @param collection collection of primitives
 * @param begin iterator to the first primitive under consideration
 * @param end iterator to the last primitive under consideration
 * @param min vertex to the minimum extent of the bounding box
 * @param max vertex to the maximum extent of the bounding box
 * @param options build options
 *
 * @return std::pair<unsigned int, blazert::Vec3r<T>> describing the cut_axis and the cut position.
 */
template<typename T, typename Iterator, template<typename> typename Collection, typename Options>
inline std::pair<unsigned int, Vec3r<T>> find_best_split_binned(const Collection<T> &collection, Iterator begin,
                                                                Iterator end, const Vec3r<T> &min, const Vec3r<T> &max,
                                                                const Options &options) {

  auto bins = std::move(sort_collection_into_bins(collection, begin, end, min, max, options));

  Vec3r<T> cut_pos;
  Vec3r<T> min_cost(std::numeric_limits<T>::max());

  for (unsigned int j = 0; j < 3; j++) {
    // Sweep left to accumulate bounding boxes and compute the right-hand side of the cost
    size_t count = 0;
    Vec3r<T> min_(std::numeric_limits<T>::max());
    Vec3r<T> max_(-std::numeric_limits<T>::max());

    for (unsigned int i = bins.size - 1; i > 0; i--) {
      Bin<T> &bin = bins.bin[j * bins.size + i];
      unity(min_, max_, bin.min, bin.max);
      count += bin.count;
      bin.cost = static_cast<T>(count) * calculate_box_surface(min_, max_);
    }

    // Sweep right to compute the full cost
    count = 0;
    min_ = std::numeric_limits<T>::max();
    max_ = -std::numeric_limits<T>::max();

    unsigned int min_bin = 1;

    for (unsigned int i = 0; i < bins.size - 1; i++) {
      Bin<T> &bin = bins.bin[j * bins.size + i];
      Bin<T> &next_bin = bins.bin[j * bins.size + i + 1];
      unity(min_, max_, bin.min, bin.max);
      count += bin.count;
      T cost = static_cast<T>(count) * calculate_box_surface(min_, max_) + next_bin.cost;

      if (cost < min_cost[j]) {
        min_cost[j] = cost;
        // Store the beginning of the right partition
        min_bin = i + 1;
      }
    }
    cut_pos[j] = static_cast<T>(min_bin) * ((max[j] - min[j]) / static_cast<T>(bins.size)) + min[j];
  }

  unsigned int min_cost_axis = 0;
  if (min_cost[0] > min_cost[1])
    min_cost_axis = 1;
  if (min_cost[min_cost_axis] > min_cost[2])
    min_cost_axis = 2;

  return std::make_pair(min_cost_axis, std::move(cut_pos));
}

}// namespace blazert

#endif// BLAZERT_BVH_BINBUFFER_H_
