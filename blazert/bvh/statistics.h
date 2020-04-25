#pragma once
#ifndef BLAZERT_BVH_STATISTICS_H_
#define BLAZERT_BVH_STATISTICS_H_

namespace blazert {

class BVHBuildStatistics {
public:
  unsigned int max_tree_depth;
  unsigned int num_leaf_nodes;
  unsigned int num_branch_nodes;
  double build_secs;

  BVHBuildStatistics()
      : max_tree_depth(0),
        num_leaf_nodes(0),
        num_branch_nodes(0),
        build_secs(0.0) {}
};
}// namespace blazert

#endif// BLAZERT_BVH_STATISTICS_H
