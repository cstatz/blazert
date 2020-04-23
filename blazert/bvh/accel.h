#pragma once
#ifndef BLAZERT_BVH_ACCEL_H_
#define BLAZERT_BVH_ACCEL_H_

#include <atomic>
#include <limits>
#include <queue>

#include <blazert/bvh/aabb.h>
#include <blazert/bvh/bbox.h>
#include <blazert/bvh/binbuffer.h>
#include <blazert/bvh/node.h>
#include <blazert/bvh/statistics.h>
#include <blazert/datatypes.h>
#include <blazert/defines.h>
#include <blazert/stack.h>

namespace blazert {

/**
 * @brief Bounding Volume Hierarchy acceleration.
 *
 * BVHAccel is central part of ray tracing(ray traversal).
 * BVHAccel takes an input geometry(primitive) information and build a data structure
 * for efficient ray tracing(`O(log2 N)` in theory, where N is the number of primitive in the scene).
 *
 * @tparam T real value type(float or double).
 */
template<typename T>
class alignas(sizeof(Vec3r<T>)) BVHAccel {
private:
  std::vector<BVHNode<T>> nodes_;
  std::vector<unsigned int> indices_;// max 4G triangles.
  std::vector<BBox<T>> bboxes_;
  BVHBuildOptions<T> options_;
  BVHBuildStatistics stats_;

public:
  BVHAccel() {}
  ~BVHAccel() {}

  ///
  /// Build BVH for input primitives.
  ///
  /// @tparam Prim Primitive(e.g. Triangle) accessor class.
  /// @tparam Pred Predicator(comparator class object for `Prim` class to find nearest hit point)
  ///
  /// @param[in] num_primitives The number of primitive.
  /// @param[in] p Primitive accessor class object.
  /// @param[in] pred Predicator object.
  ///
  /// @return true upon success.
  ///
  template<class Prim, class Pred>
  bool Build(const Prim &p, const Pred &pred, const BVHBuildOptions<T> &options = BVHBuildOptions<T>());

  ///
  /// Get statistics of built BVH tree. Valid after `Build()`
  ///
  /// @return BVH build statistics.
  ///
  BVHBuildStatistics GetStatistics() const { return stats_; }

#if defined(BLAZERT_ENABLE_SERIALIZATION)
  ///
  /// Dump built BVH to the file.
  ///
  //bool Dump(const char *filename) const;
  //bool Dump(FILE *fp) const;

  ///
  /// Load BVH binary
  ///
  //bool Load(const char *filename);
  //bool Load(FILE *fp);
#endif

  void Debug();

  ///
  /// @brief Traverse into BVH along ray and find closest hit point & primitive if
  /// found
  ///
  /// @tparam I Intersector class
  /// @tparam H Hit class
  ///
  /// @param[in] ray Input ray
  /// @param[in] intersector Intersector object. This object is called for each possible intersection of ray and BVH during traversal.
  /// @param[out] isect Intersection point information(filled when any hit point was found)
  /// @param[in] options Traversal options.
  ///
  /// @return true if any hit point found
  ///
  template<class I, class H>
  bool Traverse(const Ray<T> &ray, const I &intersector, H &isect,
                const BVHTraceOptions &options = BVHTraceOptions()) const;

  ///
  /// List up nodes which intersects along the ray.
  /// This function is useful for two-level BVH traversal.
  /// See `examples/nanosg` for example.
  ///
  /// @tparam I Intersection class
  ///
  ///
  ///
  template<class I>
  bool ListNodeIntersections(const Ray<T> &ray, unsigned int max_intersections,
                             const I &intersector,
                             StackVector<NodeHit<T>, 128> &hits) const;

  const std::vector<BVHNode<T>> &GetNodes() const { return nodes_; }
  const std::vector<unsigned int> &GetIndices() const { return indices_; }

  ///
  /// Returns bounding box of built BVH.
  ///
  void BoundingBox(Vec3r<T> &bmin, Vec3r<T> &bmax) const {
    if (nodes_.empty()) {
      bmin = std::numeric_limits<T>::max();
      bmax = -std::numeric_limits<T>::max();
    } else {
      bmin = nodes_.bmin;
      bmax = nodes_.bmax;
    }
  }

  bool IsValid() const { return nodes_.size() > 0; }

private:
  //#if defined(BLAZERT_ENABLE_PARALLEL_BUILD)
  typedef struct {
    unsigned int left_idx;
    unsigned int right_idx;
    unsigned int offset;
  } ShallowNodeInfo;

  // Used only during BVH construction
  std::vector<ShallowNodeInfo> shallow_node_infos_;

  /// Builds shallow BVH tree recursively.
  template<class P, class Pred>
  unsigned int BuildShallowTree(std::vector<BVHNode<T>> &out_nodes,
                                unsigned int left_idx, unsigned int right_idx,
                                unsigned int depth,
                                unsigned int max_shallow_depth, const P &p,
                                const Pred &pred);
  //#endif

  /// Builds BVH tree recursively.
  template<class P, class Pred>
  unsigned int BuildTree(BVHBuildStatistics &out_stat,
                         std::vector<BVHNode<T>> &out_nodes,
                         unsigned int left_idx, unsigned int right_idx,
                         unsigned int depth, const P &p, const Pred &pred);

  template<class I>
  bool TestLeafNode(const BVHNode<T> &node, const Ray<T> &ray,
                    const I &intersector) const;

  template<class I>
  bool TestLeafNodeIntersections(
      const BVHNode<T> &node, const Ray<T> &ray, const int max_intersections,
      const I &intersector,
      std::priority_queue<NodeHit<T>, std::vector<NodeHit<T>>,
                          NodeHitComparator<T>> &isect_pq) const;

  template<class P, class Pred>
  unsigned int BuildShallowTree(std::vector<T> &out_nodes, unsigned int left_idx, unsigned int right_idx, unsigned int depth, unsigned int max_shallow_depth, const P &p, const Pred &pred);
};

//
//
// --
//

#if defined(BLAZERT_ENABLE_PARALLEL_BUILD)
template<typename T>
template<class P, class Pred>
unsigned int BVHAccel<T>::BuildShallowTree(std::vector<BVHNode<T>> &out_nodes,
                                           unsigned int left_idx,
                                           unsigned int right_idx,
                                           unsigned int depth,
                                           unsigned int max_shallow_depth,
                                           const P &p, const Pred &pred) {
  assert(left_idx <= right_idx);

  unsigned int offset = static_cast<unsigned int>(out_nodes.size());

  if (stats_.max_tree_depth < depth) {
    stats_.max_tree_depth = depth;
  }

  Vec3r<T> bmin, bmax;

#ifdef BLAZERT_ENABLE_PARALLEL_BUILD
#if __cplusplus >= 201103L
  ComputeBoundingBoxThreaded(bmin, bmax, indices_, left_idx, right_idx, p);
#elif defined(_OPENMP)
  ComputeBoundingBoxOMP(bmin, bmax, indices_, left_idx, right_idx, p);
#else
  ComputeBoundingBox(bmin, bmax, indices_, left_idx, right_idx, p);
#endif
#else
  ComputeBoundingBox(bmin, bmax, indices_, left_idx, right_idx, p);
#endif

  unsigned int n = right_idx - left_idx;
  if ((n <= options_.min_leaf_primitives) || (depth >= options_.max_tree_depth)) {
    // Create leaf node.
    BVHNode<T> leaf;

    leaf.bmin = bmin;
    leaf.bmax = bmax;

    assert(left_idx < std::numeric_limits<unsigned int>::max());

    leaf.flag = 1;// leaf
    leaf.data[0] = n;
    leaf.data[1] = left_idx;

    out_nodes.push_back(leaf);// atomic update
    stats_.num_leaf_nodes++;

    return offset;
  }

  //
  // Create branch node.
  //
  if (depth >= max_shallow_depth) {
    // Delay to build tree
    ShallowNodeInfo info;
    info.left_idx = left_idx;
    info.right_idx = right_idx;
    info.offset = offset;
    shallow_node_infos_.push_back(info);

    // Add dummy node.
    BVHNode<T> node;
    node.axis = -1;
    node.flag = -1;
    out_nodes.push_back(node);

    return offset;

  } else {
    //
    // TODO(LTE): multi-threaded SAH computation, or use simple object median or
    // spacial median for shallow tree to speeding up the parallel build.
    //

    //
    // Compute SAH and find best split axis and position
    //
    Vec3r<T> cut_pos{0.};

    BinBuffer<T> bins(options_.bin_size);
    ContributeBinBuffer(bins, bmin, bmax, indices_, left_idx, right_idx, p);
    int min_cut_axis = FindCutFromBinBuffer<T>(cut_pos, bins, bmin, bmax, n, options_.cost_t_aabb);

    // Try all 3 axis until good cut position avaiable.
    unsigned int mid_idx = left_idx;
    int cut_axis = min_cut_axis;

    // TODO: This needs a rework ...
    for (int axis_try = 0; axis_try < 3; axis_try++) {
      unsigned int *begin = &indices_[left_idx];
      unsigned int *end =
          &indices_[right_idx - 1] + 1;// mimics end() iterator
      unsigned int *mid = 0;

      // try min_cut_axis first.
      cut_axis = (min_cut_axis + axis_try) % 3;

      pred.Set(cut_axis, cut_pos[cut_axis]);

      //
      // Split at (cut_axis, cut_pos)
      // indices_ will be modified.
      //
      mid = std::partition(begin, end, pred);

      mid_idx = left_idx + static_cast<unsigned int>((mid - begin));

      if ((mid_idx == left_idx) || (mid_idx == right_idx)) {
        // Can't split well.
        // Switch to object median (which may create unoptimized tree, but
        // stable)
        mid_idx = left_idx + (n >> 1);

        // Try another axis if there's an axis to try.

      } else {
        // Found good cut. exit loop.
        break;
      }
    }

    BVHNode<T> node;
    node.axis = cut_axis;
    node.flag = 0;// 0 = branch

    out_nodes.push_back(node);

    unsigned int left_child_index = 0;
    unsigned int right_child_index = 0;

    left_child_index = BuildShallowTree(out_nodes, left_idx, mid_idx, depth + 1, max_shallow_depth, p, pred);

    right_child_index = BuildShallowTree(out_nodes, mid_idx, right_idx, depth + 1, max_shallow_depth, p, pred);

    //std::cout << "shallow[" << offset << "] l and r = " << left_child_index << ", " << right_child_index << std::endl;
    out_nodes[offset].data[0] = left_child_index;
    out_nodes[offset].data[1] = right_child_index;
    out_nodes[offset].bmin = bmin;
    out_nodes[offset].bmax = bmax;
  }

  stats_.num_branch_nodes++;

  return offset;
}
#endif

template<typename T>
template<class P, class Pred>
unsigned int BVHAccel<T>::BuildTree(BVHBuildStatistics &out_stat,
                                    std::vector<BVHNode<T>> &out_nodes,
                                    unsigned int left_idx,
                                    unsigned int right_idx, unsigned int depth,
                                    const P &p, const Pred &pred) {
  assert(left_idx <= right_idx);

  unsigned int offset = static_cast<unsigned int>(out_nodes.size());

  if (out_stat.max_tree_depth < depth) {
    out_stat.max_tree_depth = depth;
  }

  Vec3r<T> bmin, bmax;
  if (!bboxes_.empty()) {
    GetBoundingBox(bmin, bmax, bboxes_, indices_, left_idx, right_idx);
  } else {
    ComputeBoundingBox(bmin, bmax, indices_, left_idx, right_idx, p);
  }

  unsigned int n = right_idx - left_idx;
  if ((n <= options_.min_leaf_primitives) || (depth >= options_.max_tree_depth)) {
    // Create leaf node.
    BVHNode<T> leaf;
    leaf.bmin = bmin;
    leaf.bmax = bmax;

    assert(left_idx < std::numeric_limits<unsigned int>::max());

    leaf.flag = 1;// leaf
    leaf.data[0] = n;
    leaf.data[1] = left_idx;

    out_nodes.push_back(leaf);// atomic update

    out_stat.num_leaf_nodes++;

    return offset;
  }

  //
  // Create branch node.
  //

  //
  // Compute SAH and find best split axis and position
  //
  Vec3r<T> cut_pos{0.0};

  BinBuffer<T> bins(options_.bin_size);
  ContributeBinBuffer(bins, bmin, bmax, indices_, left_idx, right_idx, p);
  int min_cut_axis = FindCutFromBinBuffer<T>(cut_pos, bins, bmin, bmax, n, options_.cost_t_aabb);

  // Try all 3 axis until good cut position avaiable.
  unsigned int mid_idx = left_idx;
  int cut_axis = min_cut_axis;

  for (int axis_try = 0; axis_try < 3; axis_try++) {
    unsigned int *begin = &indices_[left_idx];
    unsigned int *end = &indices_[right_idx - 1] + 1;// mimics end() iterator.
    unsigned int *mid = 0;

    // try min_cut_axis first.
    cut_axis = (min_cut_axis + axis_try) % 3;

    pred.Set(cut_axis, cut_pos[cut_axis]);

    //
    // Split at (cut_axis, cut_pos)
    // indices_ will be modified.
    //
    mid = std::partition(begin, end, pred);

    mid_idx = left_idx + static_cast<unsigned int>((mid - begin));

    if ((mid_idx == left_idx) || (mid_idx == right_idx)) {
      // Can't split well.
      // Switch to object median(which may create unoptimized tree, but
      // stable)
      mid_idx = left_idx + (n >> 1);

      // Try another axis to find better cut.

    } else {
      // Found good cut. exit loop.
      break;
    }
  }

  BVHNode<T> node;
  node.axis = cut_axis;
  node.flag = 0;// 0 = branch

  out_nodes.push_back(node);

  unsigned int left_child_index = 0;
  unsigned int right_child_index = 0;

  left_child_index =
      BuildTree(out_stat, out_nodes, left_idx, mid_idx, depth + 1, p, pred);

  right_child_index =
      BuildTree(out_stat, out_nodes, mid_idx, right_idx, depth + 1, p, pred);

  {
    out_nodes[offset].data[0] = left_child_index;
    out_nodes[offset].data[1] = right_child_index;

    out_nodes[offset].bmin = bmin;
    out_nodes[offset].bmax = bmax;
  }

  out_stat.num_branch_nodes++;

  return offset;
}

template<typename T>
template<class Prim, class Pred>
bool BVHAccel<T>::Build(const Prim &p,
                        const Pred &pred, const BVHBuildOptions<T> &options) {
  options_ = options;
  stats_ = BVHBuildStatistics();

  nodes_.clear();
  bboxes_.clear();
#if defined(BLAZERT_ENABLE_PARALLEL_BUILD)
  shallow_node_infos_.clear();
#endif

  assert(options_.bin_size > 1);

  if (p.size() == 0) {
    return false;
  }

  unsigned int n = p.size();

  //
  // 1. Create triangle indices(this will be permutated in BuildTree)
  //
  indices_.resize(n);

  {
    size_t num_threads = std::min(
        size_t(kBLAZERT_MAX_THREADS),
        std::max(size_t(1), size_t(std::thread::hardware_concurrency())));

    if (n < num_threads) {
      num_threads = n;
    }

    std::vector<std::thread> workers;

    size_t ndiv = n / num_threads;

    for (size_t t = 0; t < num_threads; t++) {
      workers.emplace_back(std::thread([&, t]() {
        size_t si = t * ndiv;
        size_t ei = (t == (num_threads - 1)) ? n : std::min((t + 1) * ndiv, size_t(n));

        for (size_t k = si; k < ei; k++) {
          indices_[k] = static_cast<unsigned int>(k);
        }
      }));
    }

    for (auto &t : workers) {
      t.join();
    }
  }

  //
  // 2. Compute bounding box (optional).
  //
  Vec3r<T> bmin, bmax;

  if (options.cache_bbox) {
    bmin = std::numeric_limits<T>::max();
    bmax = -std::numeric_limits<T>::max();

    bboxes_.resize(n);

    for (size_t i = 0; i < n; i++) {// for each primitive
      unsigned int idx = indices_[i];

      BBox<T> bbox;
      p.BoundingBox(bbox.bmin, bbox.bmax, static_cast<unsigned int>(i));
      bboxes_[idx] = bbox;

      // xyz
      for (int k = 0; k < 3; k++) {
        bmin[k] = std::min(bmin[k], bbox.bmin[k]);
        bmax[k] = std::max(bmax[k], bbox.bmax[k]);
      }
    }

  } else {
#if __cplusplus >= 201103L
    ComputeBoundingBoxThreaded(bmin, bmax, indices_, 0, n, p);
#elif defined(_OPENMP)
    ComputeBoundingBoxOMP(bmin, bmax, indices_, 0, n, p);
#else
    ComputeBoundingBox(bmin, bmax, indices_, 0, n, p);
#endif
  }

//
// 3. Build tree
//
#ifdef BLAZERT_ENABLE_PARALLEL_BUILD
#if __cplusplus >= 201103L

  // Do parallel build for large enough datasets.
  if (n > options.min_primitives_for_parallel_build) {
    BuildShallowTree(nodes_, 0, n, /* root depth */ 0, options.shallow_depth, p, pred);// [0, n)

    assert(shallow_node_infos_.size() > 0);

    // Build deeper tree in parallel
    std::vector<std::vector<BVHNode<T>>> local_nodes(
        shallow_node_infos_.size());
    std::vector<BVHBuildStatistics> local_stats(shallow_node_infos_.size());

    size_t num_threads = std::min(
        size_t(kBLAZERT_MAX_THREADS),
        std::max(size_t(1), size_t(std::thread::hardware_concurrency())));
    if (shallow_node_infos_.size() < num_threads) {
      num_threads = shallow_node_infos_.size();
    }

    std::vector<std::thread> workers;
    std::atomic<uint32_t> i(0);

    for (size_t t = 0; t < num_threads; t++) {
      workers.emplace_back(std::thread([&]() {
        uint32_t idx = 0;
        while ((idx = (i++)) < shallow_node_infos_.size()) {
          // Create thread-local copy of Pred since some mutable variables are
          // modified during SAH computation.
          const Pred local_pred = pred;
          unsigned int left_idx = shallow_node_infos_[size_t(idx)].left_idx;
          unsigned int right_idx = shallow_node_infos_[size_t(idx)].right_idx;
          BuildTree(local_stats[size_t(idx)], local_nodes[size_t(idx)], left_idx, right_idx, options.shallow_depth, p, local_pred);
        }
      }));
    }

    for (auto &t : workers) {
      t.join();
    }

    // Join local nodes
    for (size_t ii = 0; ii < local_nodes.size(); ii++) {
      assert(!local_nodes[ii].empty());
      size_t offset = nodes_.size();

      // Add offset to child index (for branch node).
      for (size_t j = 0; j < local_nodes[ii].size(); j++) {
        if (local_nodes[ii][j].flag == 0) {// branch
          local_nodes[ii][j].data[0] += offset - 1;
          local_nodes[ii][j].data[1] += offset - 1;
        }
      }

      // replace
      nodes_[shallow_node_infos_[ii].offset] = local_nodes[ii][0];

      // Skip root element of the local node.
      nodes_.insert(nodes_.end(), local_nodes[ii].begin() + 1,
                    local_nodes[ii].end());
    }

    // Join statistics
    for (size_t ii = 0; ii < local_nodes.size(); ii++) {
      stats_.max_tree_depth =
          std::max(stats_.max_tree_depth, local_stats[ii].max_tree_depth);
      stats_.num_leaf_nodes += local_stats[ii].num_leaf_nodes;
      stats_.num_branch_nodes += local_stats[ii].num_branch_nodes;
    }

  } else {
    // Single thread.
    BuildTree(stats_, nodes_, 0, n, /* root depth */ 0, p, pred);// [0, n)
  }

#elif defined(_OPENMP)

  // Do parallel build for large enough datasets.
  if (n > options.min_primitives_for_parallel_build) {
    BuildShallowTree(nodes_, 0, n, /* root depth */ 0, options.shallow_depth, p, pred);// [0, n)

    assert(shallow_node_infos_.size() > 0);

    // Build deeper tree in parallel
    std::vector<std::vector<BVHNode<T>>> local_nodes(
        shallow_node_infos_.size());
    std::vector<BVHBuildStatistics> local_stats(shallow_node_infos_.size());

#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(shallow_node_infos_.size()); i++) {
      unsigned int left_idx = shallow_node_infos_[size_t(i)].left_idx;
      unsigned int right_idx = shallow_node_infos_[size_t(i)].right_idx;
      const Pred local_pred = pred;
      BuildTree(local_stats[size_t(i)], local_nodes[size_t(i)], left_idx, right_idx, options.shallow_depth, p, local_pred);
    }

    // Join local nodes
    for (size_t i = 0; i < local_nodes.size(); i++) {
      assert(!local_nodes[size_t(i)].empty());
      size_t offset = nodes_.size();

      // Add offset to child index (for branch node).
      for (size_t j = 0; j < local_nodes[i].size(); j++) {
        if (local_nodes[i][j].flag == 0) {// branch
          local_nodes[i][j].data[0] += offset - 1;
          local_nodes[i][j].data[1] += offset - 1;
        }
      }

      // replace
      nodes_[shallow_node_infos_[i].offset] = local_nodes[i][0];

      // Skip root element of the local node.
      nodes_.insert(nodes_.end(), local_nodes[i].begin() + 1,
                    local_nodes[i].end());
    }

    // Join statistics
    for (size_t i = 0; i < local_nodes.size(); i++) {
      stats_.max_tree_depth =
          std::max(stats_.max_tree_depth, local_stats[i].max_tree_depth);
      stats_.num_leaf_nodes += local_stats[i].num_leaf_nodes;
      stats_.num_branch_nodes += local_stats[i].num_branch_nodes;
    }

  } else {
    // Single thread
    BuildTree(stats_, nodes_, 0, n, /* root depth */ 0, p, pred);// [0, n)
  }

#else// !BLAZERT_ENABLE_PARALLEL_BUILD
  {
    BuildTree(stats_, nodes_, 0, n, /* root depth */ 0, p, pred);// [0, n)
  }
#endif
#else// !_OPENMP

  // Single thread BVH build
  {
    BuildTree(stats_, nodes_, 0, n, /* root depth */ 0, p, pred);// [0, n)
  }
#endif
  return true;
}

template<typename T>
void BVHAccel<T>::Debug() {
  for (size_t i = 0; i < indices_.size(); i++) {
    printf("index[%d] = %d\n", int(i), int(indices_[i]));
  }

  for (size_t i = 0; i < nodes_.size(); i++) {
    printf("node[%d] : bmin %f, %f, %f, bmax %f, %f, %f\n", int(i),
           nodes_[i].bmin[0], nodes_[i].bmin[1], nodes_[i].bmin[1],
           nodes_[i].bmax[0], nodes_[i].bmax[1], nodes_[i].bmax[1]);
  }
}
//
//#if defined(BLAZERT_ENABLE_SERIALIZATION)
//template<typename T>
//bool BVHAccel<T>::Dump(const char *filename) const {
//  FILE *fp = fopen(filename, "wb");
//  if (!fp) {
//    // fprintf(stderr, "[BVHAccel] Cannot write a file: %s\n", filename);
//    return false;
//  }
//
//  size_t numNodes = nodes_.size();
//  assert(nodes_.size() > 0);
//
//  size_t numIndices = indices_.size();
//
//  size_t r = 0;
//  r = fwrite(&numNodes, sizeof(size_t), 1, fp);
//  assert(r == 1);
//
//  r = fwrite(&nodes_.at(0), sizeof(BVHNode<T>), numNodes, fp);
//  assert(r == numNodes);
//
//  r = fwrite(&numIndices, sizeof(size_t), 1, fp);
//  assert(r == 1);
//
//  r = fwrite(&indices_.at(0), sizeof(unsigned int), numIndices, fp);
//  assert(r == numIndices);
//
//  fclose(fp);
//
//  return true;
//}
//
//template<typename T>
//bool BVHAccel<T>::Dump(FILE *fp) const {
//  size_t numNodes = nodes_.size();
//  assert(nodes_.size() > 0);
//
//  size_t numIndices = indices_.size();
//
//  size_t r = 0;
//  r = fwrite(&numNodes, sizeof(size_t), 1, fp);
//  assert(r == 1);
//
//  r = fwrite(&nodes_.at(0), sizeof(BVHNode<T>), numNodes, fp);
//  assert(r == numNodes);
//
//  r = fwrite(&numIndices, sizeof(size_t), 1, fp);
//  assert(r == 1);
//
//  r = fwrite(&indices_.at(0), sizeof(unsigned int), numIndices, fp);
//  assert(r == numIndices);
//
//  return true;
//}
//
//template<typename T>
//bool BVHAccel<T>::Load(const char *filename) {
//  FILE *fp = fopen(filename, "rb");
//  if (!fp) {
//    // fprintf(stderr, "Cannot open file: %s\n", filename);
//    return false;
//  }
//
//  size_t numNodes;
//  size_t numIndices;
//
//  size_t r = 0;
//  r = fread(&numNodes, sizeof(size_t), 1, fp);
//  assert(r == 1);
//  assert(numNodes > 0);
//
//  nodes_.resize(numNodes);
//  r = fread(&nodes_.at(0), sizeof(BVHNode<T>), numNodes, fp);
//  assert(r == numNodes);
//
//  r = fread(&numIndices, sizeof(size_t), 1, fp);
//  assert(r == 1);
//
//  indices_.resize(numIndices);
//
//  r = fread(&indices_.at(0), sizeof(unsigned int), numIndices, fp);
//  assert(r == numIndices);
//
//  fclose(fp);
//
//  return true;
//}
//
//template<typename T>
//bool BVHAccel<T>::Load(FILE *fp) {
//  size_t numNodes;
//  size_t numIndices;
//
//  size_t r = 0;
//  r = fread(&numNodes, sizeof(size_t), 1, fp);
//  assert(r == 1);
//  assert(numNodes > 0);
//
//  nodes_.resize(numNodes);
//  r = fread(&nodes_.at(0), sizeof(BVHNode<T>), numNodes, fp);
//  assert(r == numNodes);
//
//  r = fread(&numIndices, sizeof(size_t), 1, fp);
//  assert(r == 1);
//
//  indices_.resize(numIndices);
//
//  r = fread(&indices_.at(0), sizeof(unsigned int), numIndices, fp);
//  assert(r == numIndices);
//
//  return true;
//}
//#endif

template<typename T>
template<class I>
inline bool BVHAccel<T>::TestLeafNode(const BVHNode<T> &node, const Ray<T> &ray,
                                      const I &intersector) const {
  bool hit = false;

  unsigned int num_primitives = node.data[0];
  unsigned int offset = node.data[1];

  T t = intersector.GetT();// current hit distance

  Vec3r<T> ray_org = ray.org;
  Vec3r<T> ray_dir = ray.dir;

  for (unsigned int i = 0; i < num_primitives; i++) {
    unsigned int prim_idx = indices_[i + offset];

    T local_t = t;
    if (intersector.Intersect(&local_t, prim_idx)) {
      // Update isect state
      t = local_t;

      intersector.Update(t, prim_idx);
      hit = true;
    }
  }

  return hit;
}

template<typename T>
template<class I, class H>
bool BVHAccel<T>::Traverse(const Ray<T> &ray, const I &intersector, H &isect,
                           const BVHTraceOptions &options) const {
  const int kMaxStackDepth = 512;
  //(void) kMaxStackDepth;

  T hit_t = ray.max_t;

  int node_stack_index = 0;
  unsigned int node_stack[512];
  node_stack[0] = 0;

  // Init isect info as no hit
  intersector.Update(hit_t, static_cast<unsigned int>(-1));

  intersector.PrepareTraversal(ray, options);

  Vec3i dir_sign;
  dir_sign[0] = ray.dir[0] < static_cast<T>(0.0) ? 1 : 0;
  dir_sign[1] = ray.dir[1] < static_cast<T>(0.0) ? 1 : 0;
  dir_sign[2] = ray.dir[2] < static_cast<T>(0.0) ? 1 : 0;

  Vec3r<T> ray_inv_dir;
  Vec3r<T> ray_dir = ray.dir;

  ray_inv_dir = vector_safe_inverse(ray_dir);

  Vec3r<T> ray_org = ray.org;

  T min_t = std::numeric_limits<T>::max();
  T max_t = -std::numeric_limits<T>::max();

  while (node_stack_index >= 0) {
    unsigned int index = node_stack[node_stack_index];
    const BVHNode<T> &node = nodes_[index];

    node_stack_index--;

    bool hit = IntersectRayAABB<T>(&min_t, &max_t, ray.min_t, hit_t, node.bmin,
                                node.bmax, ray_org, ray_inv_dir, dir_sign);

    if (hit) {
      // Branch node
      if (node.flag == 0) {
        int order_near = dir_sign[node.axis];
        int order_far = 1 - order_near;

        // Traverse near first.
        node_stack[++node_stack_index] = node.data[order_far];
        node_stack[++node_stack_index] = node.data[order_near];
      } else if (TestLeafNode(node, ray, intersector)) {// Leaf node
        hit_t = intersector.GetT();
      }
    }
  }

  assert(node_stack_index < kBLAZERT_MAX_STACK_DEPTH);

  bool hit = (intersector.GetT() < ray.max_t);
  intersector.PostTraversal(ray, hit, isect);

  return hit;
}

template<typename T>
template<class I>
inline bool BVHAccel<T>::TestLeafNodeIntersections(
    const BVHNode<T> &node, const Ray<T> &ray, const int max_intersections,
    const I &intersector,
    std::priority_queue<NodeHit<T>, std::vector<NodeHit<T>>,
                        NodeHitComparator<T>> &isect_pq) const {
  bool hit = false;

  unsigned int num_primitives = node.data[0];
  unsigned int offset = node.data[1];

  Vec3r<T> ray_org = ray.org;
  Vec3r<T> ray_dir = ray.dir;

  intersector.PrepareTraversal(ray);

  for (unsigned int i = 0; i < num_primitives; i++) {
    unsigned int prim_idx = indices_[i + offset];

    T min_t, max_t;

    if (intersector.Intersect(&min_t, &max_t, prim_idx)) {
      // Always add to isect lists.
      NodeHit<T> isect;
      isect.t_min = min_t;
      isect.t_max = max_t;
      isect.node_id = prim_idx;

      if (isect_pq.size() < static_cast<size_t>(max_intersections)) {
        isect_pq.push(isect);
      } else if (min_t < isect_pq.top().t_min) {
        // delete the furthest intersection and add a new intersection.
        isect_pq.pop();

        isect_pq.push(isect);
      }
    }
  }

  return hit;
}

template<typename T>
template<class I>
bool BVHAccel<T>::ListNodeIntersections(
    const Ray<T> &ray, unsigned int max_intersections, const I &intersector,
    StackVector<NodeHit<T>, 128> &hits) const {
  const int kMaxStackDepth = 512;

  T hit_t = ray.max_t;

  int node_stack_index = 0;
  unsigned int node_stack[512];
  node_stack[0] = 0;

  // Stores furthest intersection at top
  std::priority_queue<NodeHit<T>, std::vector<NodeHit<T>>,
                      NodeHitComparator<T>>
      isect_pq;

  (*hits)->clear();

  Vec3i dir_sign[3];
  dir_sign[0] = ray.dir[0] < static_cast<T>(0.0) ? 1 : 0;
  dir_sign[1] = ray.dir[1] < static_cast<T>(0.0) ? 1 : 0;
  dir_sign[2] = ray.dir[2] < static_cast<T>(0.0) ? 1 : 0;

  Vec3r<T> ray_inv_dir;
  Vec3r<T> ray_dir = ray.dir;

  ray_inv_dir = vsafe_inverse(ray_dir);

  Vec3r<T> ray_org = ray.org;

  T min_t, max_t;

  while (node_stack_index >= 0) {
    unsigned int index = node_stack[node_stack_index];
    const BVHNode<T> &node = nodes_[static_cast<size_t>(index)];

    node_stack_index--;

    bool hit = IntersectRayAABB(&min_t, &max_t, ray.min_t, hit_t, node.bmin,
                                node.bmax, ray_org, ray_inv_dir, dir_sign);

    if (hit) {
      // Branch node
      if (node.flag == 0) {
        int order_near = dir_sign[node.axis];
        int order_far = 1 - order_near;

        // Traverse near first.
        node_stack[++node_stack_index] = node.data[order_far];
        node_stack[++node_stack_index] = node.data[order_near];
      } else {// Leaf node
        TestLeafNodeIntersections(node, ray, max_intersections, intersector, isect_pq);
      }
    }
  }

  assert(node_stack_index < kMaxStackDepth);
  (void) kMaxStackDepth;

  if (!isect_pq.empty()) {
    // Store intesection in reverse order (make it frontmost order)
    size_t n = isect_pq.size();
    hits.resize(n);

    for (size_t i = 0; i < n; i++) {
      const NodeHit<T> &isect = isect_pq.top();
      hits[n - i - 1] = isect;
      isect_pq.pop();
    }

    return true;
  }

  return false;
}
}// namespace blazert

#endif// BLAZERT_BVH_ACCEL_H_