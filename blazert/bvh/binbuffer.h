
struct BinBuffer {
  explicit BinBuffer(unsigned int size) {
    bin_size = size;
    bin.resize(2 * 3 * size);
    clear();
  }

  void clear() { memset(&bin[0], 0, sizeof(size_t) * 2 * 3 * bin_size); }

  std::vector<size_t> bin;  // (min, max) * xyz * binsize
  unsigned int bin_size;
  unsigned int pad0;
};
template <typename T, class P>
inline void ContributeBinBuffer(BinBuffer *bins,  // [out]
                                const real3<T> &scene_min,
                                const real3<T> &scene_max,
                                unsigned int *indices, unsigned int left_idx,
                                unsigned int right_idx, const P &p) {
  T bin_size = static_cast<T>(bins->bin_size);

  // Calculate extent
  real3<T> scene_size, scene_inv_size;
  scene_size = scene_max - scene_min;

  for (int i = 0; i < 3; ++i) {
    assert(scene_size[i] >= static_cast<T>(0.0));

    if (scene_size[i] > static_cast<T>(0.0)) {
      scene_inv_size[i] = bin_size / scene_size[i];
    } else {
      scene_inv_size[i] = static_cast<T>(0.0);
    }
  }

  // Clear bin data
  std::fill(bins->bin.begin(), bins->bin.end(), 0);
  // memset(&bins->bin[0], 0, sizeof(2 * 3 * bins->bin_size));

  size_t idx_bmin[3];
  size_t idx_bmax[3];

  for (size_t i = left_idx; i < right_idx; i++) {
    //
    // Quantize the position into [0, BIN_SIZE)
    //
    // q[i] = (int)(p[i] - scene_bmin) / scene_size
    //
    real3<T> bmin;
    real3<T> bmax;

    p.BoundingBox(&bmin, &bmax, indices[i]);
    // GetBoundingBoxOfTriangle(&bmin, &bmax, vertices, faces, indices[i]);

    real3<T> quantized_bmin = (bmin - scene_min) * scene_inv_size;
    real3<T> quantized_bmax = (bmax - scene_min) * scene_inv_size;

    // idx is now in [0, BIN_SIZE)
    for (int j = 0; j < 3; ++j) {
      int q0 = static_cast<int>(quantized_bmin[j]);
      if (q0 < 0) q0 = 0;
      int q1 = static_cast<int>(quantized_bmax[j]);
      if (q1 < 0) q1 = 0;

      idx_bmin[j] = static_cast<unsigned int>(q0);
      idx_bmax[j] = static_cast<unsigned int>(q1);

      if (idx_bmin[j] >= bin_size)
        idx_bmin[j] = static_cast<unsigned int>(bin_size) - 1;

      if (idx_bmax[j] >= bin_size)
        idx_bmax[j] = static_cast<unsigned int>(bin_size) - 1;

      // Increment bin counter
      bins->bin[0 * (bins->bin_size * 3) +
                static_cast<size_t>(j) * bins->bin_size + idx_bmin[j]] += 1;
      bins->bin[1 * (bins->bin_size * 3) +
                static_cast<size_t>(j) * bins->bin_size + idx_bmax[j]] += 1;
    }
  }
}

template <typename T>
inline bool FindCutFromBinBuffer(T *cut_pos,        // [out] xyz
                                 int *minCostAxis,  // [out]
                                 const BinBuffer *bins, const real3<T> &bmin,
                                 const real3<T> &bmax, size_t num_primitives,
                                 T costTaabb) {      // should be in [0.0, 1.0]
  const T kEPS = std::numeric_limits<T>::epsilon();  // * epsScale;

  size_t left, right;
  real3<T> bsize, bstep;
  real3<T> bminLeft, bmaxLeft;
  real3<T> bminRight, bmaxRight;
  T saLeft, saRight, saTotal;
  T pos;
  T minCost[3];

  T costTtri = static_cast<T>(1.0) - costTaabb;

  (*minCostAxis) = 0;

  bsize = bmax - bmin;
  bstep = bsize * (static_cast<T>(1.0) / bins->bin_size);
  saTotal = CalculateSurfaceArea(bmin, bmax);

  T invSaTotal = static_cast<T>(0.0);
  if (saTotal > kEPS) {
    invSaTotal = static_cast<T>(1.0) / saTotal;
  }

  for (int j = 0; j < 3; ++j) {
    //
    // Compute SAH cost for the right side of each cell of the bbox.
    // Exclude both extreme sides of the bbox.
    //
    //  i:      0    1    2    3
    //     +----+----+----+----+----+
    //     |    |    |    |    |    |
    //     +----+----+----+----+----+
    //

    T minCostPos = bmin[j] + static_cast<T>(1.0) * bstep[j];
    minCost[j] = std::numeric_limits<T>::max();

    left = 0;
    right = num_primitives;
    bminLeft = bminRight = bmin;
    bmaxLeft = bmaxRight = bmax;

    for (int i = 0; i < static_cast<int>(bins->bin_size) - 1; ++i) {
      left += bins->bin[0 * (3 * bins->bin_size) +
                        static_cast<size_t>(j) * bins->bin_size +
                        static_cast<size_t>(i)];
      right -= bins->bin[1 * (3 * bins->bin_size) +
                         static_cast<size_t>(j) * bins->bin_size +
                         static_cast<size_t>(i)];

      assert(left <= num_primitives);
      assert(right <= num_primitives);

      //
      // Split pos bmin + (i + 1) * (bsize / BIN_SIZE)
      // +1 for i since we want a position on right side of the cell.
      //

      pos = bmin[j] + (i + static_cast<T>(1.0)) * bstep[j];
      bmaxLeft[j] = pos;
      bminRight[j] = pos;

      saLeft = CalculateSurfaceArea(bminLeft, bmaxLeft);
      saRight = CalculateSurfaceArea(bminRight, bmaxRight);

      T cost =
          SAH(left, saLeft, right, saRight, invSaTotal, costTaabb, costTtri);

      if (cost < minCost[j]) {
        //
        // Update the min cost
        //
        minCost[j] = cost;
        minCostPos = pos;
        // minCostAxis = j;
      }
    }

    cut_pos[j] = minCostPos;
  }

  // cut_axis = minCostAxis;
  // cut_pos = minCostPos;

  // Find min cost axis
  T cost = minCost[0];
  (*minCostAxis) = 0;

  if (cost > minCost[1]) {
    (*minCostAxis) = 1;
    cost = minCost[1];
  }
  if (cost > minCost[2]) {
    (*minCostAxis) = 2;
    cost = minCost[2];
  }

  return true;
}

template <typename T>
inline T CalculateSurfaceArea(const real3<T> &min, const real3<T> &max) {
  real3<T> box = max - min;
  return static_cast<T>(2.0) *
         (box[0] * box[1] + box[1] * box[2] + box[2] * box[0]);
}

