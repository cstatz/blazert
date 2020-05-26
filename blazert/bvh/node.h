#pragma once
#ifndef BLAZERT_BVH_NODE_H_
#define BLAZERT_BVH_NODE_H_

#include <blazert/datatypes.h>

namespace blazert {

template<typename T>
class BLAZERTALIGN BVHNode {
public:
  BVHNode() {}
  BVHNode(const BVHNode &rhs) {
    min = rhs.min;
    max = rhs.max;
    flag = rhs.flag;
    axis = rhs.axis;
    data = rhs.data;
  }

  BVHNode &operator=(const BVHNode &rhs) {
    min = rhs.min;
    max = rhs.max;
    flag = rhs.flag;
    axis = rhs.axis;
    data = rhs.data;
    return (*this);
  }

  ~BVHNode() {}

  Vec3r<T> min;
  Vec3r<T> max;

  unsigned int flag = -1;// 1 = leaf node, 0 = branch node
  unsigned int axis = -1;
  vec2ui data;  // 0->left_idx, 1->right_idx (or n prims)
};

}// namespace blazert

#endif// BLAZERT_BVH_NODE_H
