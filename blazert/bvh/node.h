#pragma once
#ifndef BLAZERT_BVH_NODE_H_
#define BLAZERT_BVH_NODE_H_

#include <blazert/datatypes.h>

namespace blazert {

/**
 * BVHnode describes a single node in the BVH for a certain primitive type.
 *
 * @tparam T floating point type
 * @tparam Collection primitive collection type
 */
template<typename T, template<typename> typename Collection>
class BLAZERTALIGN BVHNode {
public:
  typedef typename Collection<T>::primitive_type Primitives;

  BVHNode()
      : leaf(static_cast<unsigned int>(-1)),
        axis(static_cast<unsigned int>(-1)), children{static_cast<unsigned int>(-1), static_cast<unsigned int>(-1)} {}
  BVHNode(BVHNode &&rhs) noexcept
      : min(std::move(rhs.min)), max(std::move(rhs.max)), leaf(std::exchange(rhs.leaf, static_cast<unsigned int>(-1))),
        axis(std::exchange(rhs.axis, static_cast<unsigned int>(-1))),
        children{std::exchange(rhs.children[0], static_cast<unsigned int>(-1)),
                 std::exchange(rhs.children[1], static_cast<unsigned int>(-1))},
        primitives(std::move(rhs.primitives)) {}

  BVHNode(const BVHNode &rhs) = delete;
  BVHNode &operator=(const BVHNode &rhs) = delete;
  ~BVHNode() = default;

  Vec3r<T> min;
  Vec3r<T> max;

  unsigned int leaf;// 1 = leaf node, 0 = branch node
  unsigned int axis;
  unsigned int children[2];
  std::vector<Primitives> primitives;
};

template<typename T, template<typename> typename Collection>
std::ostream &operator<<(std::ostream &stream, const BVHNode<T, Collection> &node) {
  stream << "{\n";
  stream << R"(  "node": )" << &node << ",\n";
  stream << R"(  "min:" [)" << node.min[0] << ", " << node.min[1] << ", " << node.min[2] << "],\n";
  stream << R"(  "max:" [)" << node.max[0] << ", " << node.max[1] << ", " << node.max[2] << "],\n";
  stream << R"(  "leaf": )" << node.leaf << ",\n";
  stream << R"(  "axis": )" << node.axis << "\n";
  stream << "}\n";
  return stream;
}

}// namespace blazert

#endif// BLAZERT_BVH_NODE_H
