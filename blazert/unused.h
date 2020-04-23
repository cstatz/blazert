
template <class H>
class IntersectComparator {
 public:
  bool operator()(const H &a, const H &b) const { return a.t < b.t; }
};

// Predefined SAH predicator for triangle.
template <typename T = float>
class TriangleSAHPred {
 public:
  TriangleSAHPred(
      const T *vertices, const unsigned int *faces,
      size_t vertex_stride_bytes)  // e.g. 12 for sizeof(float) * XYZ
      : axis_(0),
        pos_(static_cast<T>(0.0)),
        vertices_(vertices),
        faces_(faces),
        vertex_stride_bytes_(vertex_stride_bytes) {}

  TriangleSAHPred(const TriangleSAHPred<T> &rhs)
      : axis_(rhs.axis_),
        pos_(rhs.pos_),
        vertices_(rhs.vertices_),
        faces_(rhs.faces_),
        vertex_stride_bytes_(rhs.vertex_stride_bytes_) {}

  TriangleSAHPred<T> &operator=(const TriangleSAHPred<T> &rhs) {
    axis_ = rhs.axis_;
    pos_ = rhs.pos_;
    vertices_ = rhs.vertices_;
    faces_ = rhs.faces_;
    vertex_stride_bytes_ = rhs.vertex_stride_bytes_;

    return (*this);
  }

  void Set(int axis, T pos) const {
    axis_ = axis;
    pos_ = pos;
  }

  bool operator()(unsigned int i) const {
    int axis = axis_;
    T pos = pos_;

    unsigned int i0 = faces_[3 * i + 0];
    unsigned int i1 = faces_[3 * i + 1];
    unsigned int i2 = faces_[3 * i + 2];

    real3<T> p0(get_vertex_addr<T>(vertices_, i0, vertex_stride_bytes_));
    real3<T> p1(get_vertex_addr<T>(vertices_, i1, vertex_stride_bytes_));
    real3<T> p2(get_vertex_addr<T>(vertices_, i2, vertex_stride_bytes_));

    T center = p0[axis] + p1[axis] + p2[axis];

    return (center < pos * static_cast<T>(3.0));
  }

 private:
  mutable int axis_;
  mutable T pos_;
  const T *vertices_;
  const unsigned int *faces_;
  const size_t vertex_stride_bytes_;
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

