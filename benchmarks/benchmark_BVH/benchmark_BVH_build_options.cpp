//
// Created by ogarten on 20/05/2020.
//

#include "../benchmark_helper/OriginSphere.h"
#include <benchmark/benchmark.h>

#include <blazert/blazert.h>
#include <blazert/primitives/trimesh.h>

#include <embree3/rtcore.h>

using namespace blazert;

template<typename T>
static void BM_BLAZERT_TRAVERSE_BVH_Sphere7_Tree_Depth(benchmark::State &state) {
  BVHBuildOptions<T> build_options;
  build_options.max_tree_depth = state.range(0);

  auto os = OriginSphere<T>(7);

  TriangleMesh triangles(os.vertices, os.triangles);
  BVH triangles_bvh(triangles);
  SAHBinnedBuilder builder;

  auto statistics = builder.build(triangles_bvh, build_options);
  //std::cout << "success = " << success << "\n";

  constexpr int height = 4 * 2048;
  constexpr int width = 4 * 2048;
  for (auto _ : state) {
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        const blazert::Ray<T> ray{
            {0.0, 0.0, 10.0},
            {static_cast<T>((x / T(width)) - 0.5), static_cast<T>((y / T(height)) - 0.5), T(-1.)}};
        RayHit<T> temp_rayhit;
        traverse(triangles_bvh, ray, temp_rayhit);
      }
    }
  }
}
//BENCHMARK_TEMPLATE(BM_BLAZERT_TRAVERSE_BVH_Sphere7_Tree_Depth, float)->RangeMultiplier(2)->Range(128, 4096)->Unit(benchmark::kMillisecond);
//BENCHMARK_TEMPLATE(BM_BLAZERT_TRAVERSE_BVH_Sphere7_Tree_Depth, double)->RangeMultiplier(2)->Range(128, 4096)->Unit(benchmark::kMillisecond);

template<typename T>
static void BM_BLAZERT_TRAVERSE_BVH_Sphere7_Bin_Size(benchmark::State &state) {
  BVHBuildOptions<T> build_options;
  build_options.bin_size = state.range(0);

  auto os = OriginSphere<T>(7);

  TriangleMesh triangles(os.vertices, os.triangles);

  BVH triangles_bvh(triangles);
  SAHBinnedBuilder builder;
  auto statistics = triangles_bvh.build(triangles, build_options);
  //std::cout << "success = " << success << "\n";

  constexpr int height = 4 * 2048;
  constexpr int width = 4 * 2048;
  for (auto _ : state) {
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        const blazert::Ray<T> ray{
            {0.0, 0.0, 10.0},
            {static_cast<T>((x / T(width)) - 0.5), static_cast<T>((y / T(height)) - 0.5), T(-1.)}};
        RayHit<T> temp_rayhit;
        traverse(triangles_bvh, ray, temp_rayhit);
      }
    }
  }
}
//BENCHMARK_TEMPLATE(BM_BLAZERT_TRAVERSE_BVH_Sphere7_Bin_Size, float)->RangeMultiplier(2)->Range(8, 512)->Unit(benchmark::kMillisecond);
//BENCHMARK_TEMPLATE(BM_BLAZERT_TRAVERSE_BVH_Sphere7_Bin_Size, double)->RangeMultiplier(2)->Range(8, 512)->Unit(benchmark::kMillisecond);