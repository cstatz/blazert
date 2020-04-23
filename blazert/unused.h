
template <class H>
class IntersectComparator {
 public:
  bool operator()(const H &a, const H &b) const { return a.t < b.t; }
};


template <typename T>
inline void GetBoundingBoxOfTriangle(real3<T> *bmin, real3<T> *bmax,
                                     const T *vertices,
                                     const unsigned int *faces,
                                     unsigned int index) {
  unsigned int f0 = faces[3 * index + 0];
  unsigned int f1 = faces[3 * index + 1];
  unsigned int f2 = faces[3 * index + 2];

  real3<T> p[3];

  p[0] = real3<T>(&vertices[3 * f0]);
  p[1] = real3<T>(&vertices[3 * f1]);
  p[2] = real3<T>(&vertices[3 * f2]);

  (*bmin) = p[0];
  (*bmax) = p[0];

  for (int i = 1; i < 3; i++) {
    (*bmin)[0] = std::min((*bmin)[0], p[i][0]);
    (*bmin)[1] = std::min((*bmin)[1], p[i][1]);
    (*bmin)[2] = std::min((*bmin)[2], p[i][2]);

    (*bmax)[0] = std::max((*bmax)[0], p[i][0]);
    (*bmax)[1] = std::max((*bmax)[1], p[i][1]);
    (*bmax)[2] = std::max((*bmax)[2], p[i][2]);
  }
}

