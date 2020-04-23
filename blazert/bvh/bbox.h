
///
/// @brief Bounding box.
///
template <typename T>
class BBox {
 public:
  real3<T> bmin;
  real3<T> bmax;

  BBox() {
    bmin[0] = bmin[1] = bmin[2] = std::numeric_limits<T>::max();
    bmax[0] = bmax[1] = bmax[2] = -std::numeric_limits<T>::max();
  }
};

template <typename T>
inline void GetBoundingBox(real3<T> *bmin, real3<T> *bmax,
                           const std::vector<BBox<T> > &bboxes,
                           unsigned int *indices, unsigned int left_index,
                           unsigned int right_index) {
  unsigned int i = left_index;
  unsigned int idx = indices[i];

  (*bmin)[0] = bboxes[idx].bmin[0];
  (*bmin)[1] = bboxes[idx].bmin[1];
  (*bmin)[2] = bboxes[idx].bmin[2];
  (*bmax)[0] = bboxes[idx].bmax[0];
  (*bmax)[1] = bboxes[idx].bmax[1];
  (*bmax)[2] = bboxes[idx].bmax[2];

  // for each face
  for (i = left_index + 1; i < right_index; i++) {
    idx = indices[i];

    // xyz
    for (int k = 0; k < 3; k++) {
      (*bmin)[k] = std::min((*bmin)[k], bboxes[idx].bmin[k]);
      (*bmax)[k] = std::max((*bmax)[k], bboxes[idx].bmax[k]);
    }
  }
}

template <typename T, class P>
inline void ComputeBoundingBox(real3<T> *bmin, real3<T> *bmax,
                               const unsigned int *indices,
                               unsigned int left_index,
                               unsigned int right_index, const P &p) {
  unsigned int idx = indices[left_index];
  p.BoundingBox(bmin, bmax, idx);

  {
    // for each primitive
    for (unsigned int i = left_index + 1; i < right_index; i++) {
      idx = indices[i];
      real3<T> bbox_min, bbox_max;
      p.BoundingBox(&bbox_min, &bbox_max, idx);

      // xyz
      for (int k = 0; k < 3; k++) {
        (*bmin)[k] = std::min((*bmin)[k], bbox_min[k]);
        (*bmax)[k] = std::max((*bmax)[k], bbox_max[k]);
      }
    }
  }
}



#ifdef BLAZERT_USE_CPP11_FEATURE
template <typename T, class P>
inline void ComputeBoundingBoxThreaded(real3<T> *bmin, real3<T> *bmax,
                                       const unsigned int *indices,
                                       unsigned int left_index,
                                       unsigned int right_index, const P &p) {
  unsigned int n = right_index - left_index;

  size_t num_threads = std::min(
      size_t(kBLAZERT_MAX_THREADS),
      std::max(size_t(1), size_t(std::thread::hardware_concurrency())));

  if (n < num_threads) {
    num_threads = n;
  }

  std::vector<std::thread> workers;

  size_t ndiv = n / num_threads;

  std::vector<T> local_bmins(3 * num_threads);  // 3 = xyz
  std::vector<T> local_bmaxs(3 * num_threads);  // 3 = xyz

  for (size_t t = 0; t < num_threads; t++) {
    workers.emplace_back(std::thread([&, t]() {
      size_t si = left_index + t * ndiv;
      size_t ei = (t == (num_threads - 1)) ? size_t(right_index) : std::min(left_index + (t + 1) * ndiv, size_t(right_index));

      local_bmins[3 * t + 0] = std::numeric_limits<T>::infinity();
      local_bmins[3 * t + 1] = std::numeric_limits<T>::infinity();
      local_bmins[3 * t + 2] = std::numeric_limits<T>::infinity();
      local_bmaxs[3 * t + 0] = -std::numeric_limits<T>::infinity();
      local_bmaxs[3 * t + 1] = -std::numeric_limits<T>::infinity();
      local_bmaxs[3 * t + 2] = -std::numeric_limits<T>::infinity();

      // for each face
      for (size_t i = si; i < ei; i++) {
        unsigned int idx = indices[i];

        real3<T> bbox_min, bbox_max;
        p.BoundingBox(&bbox_min, &bbox_max, idx);

        // xyz
        for (size_t k = 0; k < 3; k++) {
          local_bmins[3 * t + k] =
              std::min(local_bmins[3 * t + k], bbox_min[int(k)]);
          local_bmaxs[3 * t + k] =
              std::max(local_bmaxs[3 * t + k], bbox_max[int(k)]);
        }
      }
    }));
  }

  for (auto &t : workers) {
    t.join();
  }

  // merge bbox
  for (size_t k = 0; k < 3; k++) {
    (*bmin)[int(k)] = local_bmins[k];
    (*bmax)[int(k)] = local_bmaxs[k];
  }

  for (size_t t = 1; t < num_threads; t++) {
    for (size_t k = 0; k < 3; k++) {
      (*bmin)[int(k)] = std::min((*bmin)[int(k)], local_bmins[3 * t + k]);
      (*bmax)[int(k)] = std::max((*bmax)[int(k)], local_bmaxs[3 * t + k]);
    }
  }
}

#ifdef _OPENMP
template <typename T, class P>
void ComputeBoundingBoxOMP(real3<T> *bmin, real3<T> *bmax,
                           const unsigned int *indices, unsigned int left_index,
                           unsigned int right_index, const P &p) {
  { p.BoundingBox(bmin, bmax, indices[left_index]); }

  T local_bmin[3] = {(*bmin)[0], (*bmin)[1], (*bmin)[2]};
  T local_bmax[3] = {(*bmax)[0], (*bmax)[1], (*bmax)[2]};

  unsigned int n = right_index - left_index;

#pragma omp parallel firstprivate(local_bmin, local_bmax) if (n > (1024 * 128))
  {
#pragma omp parallel for
    // for each face
    for (int i = int(left_index); i < int(right_index); i++) {
      unsigned int idx = indices[i];

      real3<T> bbox_min, bbox_max;

      p.BoundingBox(&bbox_min, &bbox_max, idx);

      // xyz
      for (int k = 0; k < 3; k++) {
        (*bmin)[k] = std::min((*bmin)[k], bbox_min[k]);
        (*bmax)[k] = std::max((*bmax)[k], bbox_max[k]);
      }
    }

#pragma omp critical
    {
      for (int k = 0; k < 3; k++) {
        (*bmin)[k] = std::min((*bmin)[k], local_bmin[k]);
        (*bmax)[k] = std::max((*bmax)[k], local_bmax[k]);
      }
    }
  }
}
