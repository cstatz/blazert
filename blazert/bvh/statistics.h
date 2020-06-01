#pragma once
#ifndef BLAZERT_BVH_STATISTICS_H_
#define BLAZERT_BVH_STATISTICS_H_

namespace blazert {

template<typename T>
class BVHBuildStatistics {
public:
  unsigned int max_tree_depth;
  unsigned int num_leaf_nodes;
  unsigned int num_branch_nodes;
  T build_secs;

  BVHBuildStatistics()
      : max_tree_depth(0),
        num_leaf_nodes(0),
        num_branch_nodes(0),
        build_secs(T(0.0)) {}
};

template<typename T>
std::ostream& operator<<(std::ostream& stream, const BVHBuildStatistics<T>& stats) {
  stream << "{\n";
  stream << "  statistics: " << &stats << ",\n";
  stream << "  max_tree_depth: " << stats.max_tree_depth << ",\n";
  //stream << "  axis: " << node.axis << "\n";
  stream << "}\n";
  return stream;
}
}// namespace blazert

#endif// BLAZERT_BVH_STATISTICS_H
