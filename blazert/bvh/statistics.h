#pragma once
#ifndef BLAZERT_BVH_STATISTICS_H_
#define BLAZERT_BVH_STATISTICS_H_

#include <iostream>
#include <chrono>

namespace blazert {

template<typename T>
class BVHBuildStatistics {

private:
  std::chrono::time_point<std::chrono::steady_clock> _start;
public:
  unsigned int max_tree_depth;
  unsigned int leaf_nodes;
  unsigned int max_num_prims_per_leaf;
  unsigned int min_num_prims_per_leaf;
  unsigned int branch_nodes;
  unsigned int bad_splits;
  std::chrono::duration<T> build_time;

  BVHBuildStatistics()
      : max_tree_depth(0),
        leaf_nodes(0),
        max_num_prims_per_leaf(0),
        min_num_prims_per_leaf(-1),
        branch_nodes(0),
        bad_splits(0),
        build_time(T(0.0)) {}

  void start() {
    _start = std::chrono::steady_clock::now();
  }

  void stop() {
    build_time = std::chrono::steady_clock::now() - _start;
  }

  void primitives_per_leaf(const unsigned int prims) {
    max_num_prims_per_leaf = std::max(prims, max_num_prims_per_leaf);
    min_num_prims_per_leaf = std::min(prims, min_num_prims_per_leaf);
  }

};

template<typename T>
std::ostream& operator<<(std::ostream& stream, const BVHBuildStatistics<T>& stats) {
  stream << "{\n";
  stream << "  statistics: " << &stats << ",\n";
  stream << "  max_tree_depth: " << stats.max_tree_depth << ",\n";
  stream << "  leaf_nodes: " << stats.leaf_nodes << ",\n";
  stream << "  max_num_prims_per_leaf: " << stats.max_num_prims_per_leaf << ",\n";
  stream << "  min_num_prims_per_leaf: " << stats.min_num_prims_per_leaf << ",\n";
  stream << "  leaf_nodes: " << stats.leaf_nodes << ",\n";
  stream << "  branch_nodes: " << stats.branch_nodes << ",\n";
  stream << "  bad_splits: " << stats.bad_splits << ",\n";
  stream << "  build_time: " << stats.build_time.count() << "\n"; // TODO: This needs special care and is implementen in C++20
  stream << "}\n";
  return stream;
}
}// namespace blazert

#endif// BLAZERT_BVH_STATISTICS_H
