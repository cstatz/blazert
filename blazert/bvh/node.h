#pragma once
#ifndef BLAZERT_BVH_NODE_H_
#define BLAZERT_BVH_NODE_H_

#include <blazert/datatypes.h>

namespace blazert {

template<typename T>
class BLAZERTALIGN BVHNode {
public:
  BVHNode() =default;
  BVHNode(BVHNode&& rhs) noexcept: min(std::move(rhs.min)), max(std::move(rhs.max)),
                                    leaf(std::exchange(rhs.leaf, -1)),
                                   axis(std::exchange(rhs.axis,-1)),
                                   //children{std::exchange(rhs.children[0], nullptr), std::exchange(rhs.children[1], nullptr)},
                                   children{std::exchange(rhs.children[0], -1), std::exchange(rhs.children[1], -1)},
                                   tris(std::move(rhs.tris)){}

  BVHNode(const BVHNode& rhs) =delete;
  BVHNode &operator=(const BVHNode &rhs) =delete;
  ~BVHNode() =default;

  Vec3r<T> min;
  Vec3r<T> max;

  unsigned int leaf = -1; // 1 = leaf node, 0 = branch node
  unsigned int axis = -1;
  //BVHNode<T> *children[2];
  unsigned int children[2];
  std::vector<Tri<T>> tris;
};

template<typename T>
std::ostream& operator<<(std::ostream& stream, const BVHNode<T>& node) {
  stream << "{\n";
  stream << "  node: " << &node << ",\n";
  stream << "  min: [" << node.min[0] << ", " << node.min[1] << ", " << node.min[2]  << "],\n";
  stream << "  max: [" << node.max[0] << ", " << node.max[1] << ", " << node.max[2]  << "],\n";
  stream << "  leaf: " << node.leaf << ",\n";
  stream << "  axis: " << node.axis << "\n";
  stream << "}\n";
  return stream;
}

}// namespace blazert

#endif// BLAZERT_BVH_NODE_H
