

template <typename T = float>
class BVHNode {
 public:
  BVHNode() {}
  BVHNode(const BVHNode &rhs) {
    bmin[0] = rhs.bmin[0];
    bmin[1] = rhs.bmin[1];
    bmin[2] = rhs.bmin[2];
    flag = rhs.flag;

    bmax[0] = rhs.bmax[0];
    bmax[1] = rhs.bmax[1];
    bmax[2] = rhs.bmax[2];
    axis = rhs.axis;

    data[0] = rhs.data[0];
    data[1] = rhs.data[1];
  }

  BVHNode &operator=(const BVHNode &rhs) {
    bmin[0] = rhs.bmin[0];
    bmin[1] = rhs.bmin[1];
    bmin[2] = rhs.bmin[2];
    flag = rhs.flag;

    bmax[0] = rhs.bmax[0];
    bmax[1] = rhs.bmax[1];
    bmax[2] = rhs.bmax[2];
    axis = rhs.axis;

    data[0] = rhs.data[0];
    data[1] = rhs.data[1];

    return (*this);
  }

  ~BVHNode() {}

  T bmin[3];
  T bmax[3];

  int flag;  // 1 = leaf node, 0 = branch node
  int axis;

  // leaf
  //   data[0] = npoints
  //   data[1] = index
  //
  // branch
  //   data[0] = child[0]
  //   data[1] = child[1]
  unsigned int data[2];
};
///
/// @brief Hit class for traversing nodes.
///
/// Stores hit information of node traversal.
/// Node traversal is used for two-level ray tracing(efficient ray traversal of a scene hierarchy)
///
template <typename T>
class NodeHit {
 public:
  NodeHit()
      : t_min(std::numeric_limits<T>::max()),
        t_max(-std::numeric_limits<T>::max()),
        node_id(static_cast<unsigned int>(-1)) {}

  NodeHit(const NodeHit<T> &rhs) {
    t_min = rhs.t_min;
    t_max = rhs.t_max;
    node_id = rhs.node_id;
  }

  NodeHit &operator=(const NodeHit<T> &rhs) {
    t_min = rhs.t_min;
    t_max = rhs.t_max;
    node_id = rhs.node_id;

    return (*this);
  }

  ~NodeHit() {}

  T t_min;
  T t_max;
  unsigned int node_id;
};

///
/// @brief Comparator object for NodeHit.
///
/// Comparator object for finding nearest hit point in node traversal.
///
template <typename T>
class NodeHitComparator {
 public:
  inline bool operator()(const NodeHit<T> &a, const NodeHit<T> &b) {
    return a.t_min < b.t_min;
  }
};
