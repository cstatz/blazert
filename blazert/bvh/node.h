#pragma once
#ifndef BLAZERT_BVH_NODE_H_
#define BLAZERT_BVH_NODE_H_

#include <blazert/datatypes.h>

namespace blazert {

template<typename T>
class alignas(sizeof(Vec3r<T>)) BVHNode {
public:
  BVHNode() {}
  BVHNode(const BVHNode &rhs) {
    bmin = rhs.bmin;
    bmax = rhs.bmax;
    flag = rhs.flag;
    axis = rhs.axis;
    data = rhs.data;
  }

  BVHNode &operator=(const BVHNode &rhs) {
    bmin = rhs.bmin;
    bmax = rhs.bmax;
    flag = rhs.flag;
    axis = rhs.axis;
    data = rhs.data;
    return (*this);
  }

  ~BVHNode() {}

  Vec3r<T> bmin;
  Vec3r<T> bmax;

  unsigned int flag = -1;// 1 = leaf node, 0 = branch node
  unsigned int axis = -1;
  vec2ui data;
};

/**
 * @brief Hit class for traversing nodes.
 *
 * Stores hit information of node traversal.
 * Node traversal is used for two-level ray tracing(efficient ray traversal of a scene hierarchy)
 */
template<typename T>
class alignas(sizeof(Vec3r<T>)) NodeHit {
public:
  NodeHit()
      : t_min(std::numeric_limits<T>::max()),
        t_max(-std::numeric_limits<T>::max()),
        node_id(static_cast<unsigned int>(-1)) {}

  inline NodeHit(const NodeHit<T> &rhs) {
    t_min = rhs.t_min;
    t_max = rhs.t_max;
    node_id = rhs.node_id;
  }

  inline NodeHit &operator=(const NodeHit<T> &rhs) {
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

/**
 * @brief Comparator object for NodeHit.
 *
 * Comparator object for finding nearest hit point in node traversal.
 */
template<typename T>
class NodeHitComparator {
public:
  inline bool operator() (const NodeHit<T> &a, const NodeHit<T> &b) const {
    return a.t_min < b.t_min;
  }
};
}// namespace blazert

#endif// BLAZERT_BVH_NODE_H
