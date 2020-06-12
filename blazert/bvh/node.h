#pragma once
#ifndef BLAZERT_BVH_NODE_H_
#define BLAZERT_BVH_NODE_H_

#include <blazert/datatypes.h>

namespace blazert {

template<typename T, template<typename> typename Collection>
class BLAZERTALIGN BVHNode {
public:
  typedef typename Collection<T>::primitive_type Primitives;

  BVHNode()
      : leaf(static_cast<unsigned int>(-1)),
        axis(static_cast<unsigned int>(-1)), children{nullptr, nullptr} {}
  BVHNode(BVHNode &&rhs) noexcept
      : min(std::move(rhs.min)), max(std::move(rhs.max)), leaf(std::exchange(rhs.leaf, static_cast<unsigned int>(-1))),
        axis(std::exchange(rhs.axis, static_cast<unsigned int>(-1))),
        children{std::exchange(rhs.children[0], nullptr), std::exchange(rhs.children[1], nullptr)},
        primitives(std::move(rhs.primitives)) {}

  BVHNode(const BVHNode &rhs) = delete;
  BVHNode &operator=(const BVHNode &rhs) = delete;
  ~BVHNode() = default;

  Vec3r<T> min;
  Vec3r<T> max;

  unsigned int leaf;// 1 = leaf node, 0 = branch node
  unsigned int axis;
  BVHNode *children[2];
  std::vector<Primitives> primitives;
};

template<typename T, template<typename> typename Collection>
std::ostream &operator<<(std::ostream &stream, const BVHNode<T, Collection> &node) {
  stream << "{\n";
  stream << "  node: " << &node << ",\n";
  stream << "  min: [" << node.min[0] << ", " << node.min[1] << ", " << node.min[2] << "],\n";
  stream << "  max: [" << node.max[0] << ", " << node.max[1] << ", " << node.max[2] << "],\n";
  stream << "  leaf: " << node.leaf << ",\n";
  stream << "  axis: " << node.axis << "\n";
  stream << "}\n";
  return stream;
}

}// namespace blazert

#endif// BLAZERT_BVH_NODE_H
