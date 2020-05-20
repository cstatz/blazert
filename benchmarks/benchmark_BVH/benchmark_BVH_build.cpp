//
// Created by ogarten on 19/05/2020.
//

#include <benchmark/benchmark.h>
#include "../benchmark_helper/OriginSphere.h"

#include <blazert/blazert.h>
#include <blazert/primitives/trimesh.h>

#include <embree3/rtcore.h>

using namespace blazert;

template<typename T>
static void BM_BLAZERT_BUILD_BVH_OriginSphere(benchmark::State& state)
{
  BVHBuildOptions<T> build_options;
  build_options.cache_bbox = false;
  BVH<T> triangles_bvh;

  const auto os = OriginSphere<T>(state.range(0));
  //std::cout << sizeof(Vec3r<float>) << "\n";
  TriangleMesh triangles(os.vertices, os.triangles);
  TriangleSAHPred triangles_sah(os.vertices, os.triangles);

  for (auto _ : state) {
    triangles_bvh.build(triangles, triangles_sah, build_options);
  }
}
//BENCHMARK_TEMPLATE(BM_BLAZERT_BUILD_BVH_OriginSphere, float)->DenseRange(2,8,1)->Unit(benchmark::kMillisecond);
//BENCHMARK_TEMPLATE(BM_BLAZERT_BUILD_BVH_OriginSphere, double)->DenseRange(2,8,1)->Unit(benchmark::kMillisecond);

static void
BM_EMBREE_BUILD_BVH_OriginSphere(benchmark::State& state)
{
  auto device = rtcNewDevice("verbose=0,start_threads=1,threads=1,set_affinity=1");
  auto scene =  rtcNewScene(device);

  constexpr const int bytestride_int = sizeof(Vec3ui) / 4 * sizeof(Vec3ui::ElementType);
  constexpr const int bytestride_float = sizeof(Vec3r<float>) / 4 * sizeof(Vec3r<float>::ElementType);

  const auto os = OriginSphere<float>(state.range(0));

  auto geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetSharedGeometryBuffer(geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, (void *) (os.triangles.data()), 0, bytestride_int, os.triangles.size());
  rtcSetSharedGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, (void *) (os.vertices.data()), 0, bytestride_float, os.vertices.size());
  rtcCommitGeometry(geometry);
  rtcAttachGeometry(scene, geometry);

  for (auto _ : state) {
    rtcCommitScene(scene);
  }
}
//BENCHMARK(BM_EMBREE_BUILD_BVH_OriginSphere)->DenseRange(2,8,1)->Unit(benchmark::kMillisecond);