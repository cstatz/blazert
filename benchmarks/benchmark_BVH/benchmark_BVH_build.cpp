//
// Created by ogarten on 19/05/2020.
//

#include <benchmark/benchmark.h>
#include "../benchmark_helper/OriginSphere.h"

#include <blazert/blazert.h>
#include <blazert/primitives/trimesh.h>

#include <third_party/nanort/nanort.h>

#include <third_party/bvh/include/bvh/bvh.hpp>
#include <third_party/bvh/include/bvh/intersectors.hpp>
#include <third_party/bvh/include/bvh/ray.hpp>
#include <third_party/bvh/include/bvh/single_ray_traverser.hpp>
#include <third_party/bvh/include/bvh/sweep_sah_builder.hpp>
#include <third_party/bvh/include/bvh/binned_sah_builder.hpp>
#include <third_party/bvh/include/bvh/triangle.hpp>
#include <third_party/bvh/include/bvh/vector.hpp>

#include <embree3/rtcore.h>

using namespace blazert;

template<typename T>
static void BM_BLAZERT_BUILD_BVH_Sphere(benchmark::State& state)
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
BENCHMARK_TEMPLATE(BM_BLAZERT_BUILD_BVH_Sphere, float)->DenseRange(2,9,1)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(BM_BLAZERT_BUILD_BVH_Sphere, double)->DenseRange(2,9,1)->Unit(benchmark::kMillisecond);

static void
BM_EMBREE_BUILD_BVH_Sphere(benchmark::State& state)
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
BENCHMARK(BM_EMBREE_BUILD_BVH_Sphere)->DenseRange(2,8,1)->Unit(benchmark::kMillisecond);

template<typename T>
static void BM_nanoRT_BUILD_BHV_Sphere(benchmark::State &state) {
  nanort::BVHBuildOptions<T> build_options;
  nanort::BVHTraceOptions trace_options;

  auto os = OriginSphere<T>(state.range(0));

  const Vec3r<T> *verts = os.vertices.data();
  const Vec3ui *tris = os.triangles.data();

  nanort::TriangleMesh<T> triangles{verts->data(), tris->data(), sizeof(Vec3r<T>)};
  nanort::TriangleSAHPred<T> triangles_sah{verts->data(), tris->data(), sizeof(Vec3r<T>)};

  nanort::BVHAccel<T> triangles_bvh;
  for (auto _ : state) {
    triangles_bvh.Build(os.triangle_count(), triangles, triangles_sah, build_options);
  }
}
BENCHMARK_TEMPLATE(BM_nanoRT_BUILD_BHV_Sphere, float)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(BM_nanoRT_BUILD_BHV_Sphere, double)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);

template<typename T>
static void BM_bvh_BUILD_BHV_Sphere_SweepSAH(benchmark::State &state) {
  using Scalar = T;
  using Vector3 = bvh::Vector3<Scalar>;
  using Triangle = bvh::Triangle<Scalar>;
  using Ray = bvh::Ray<Scalar>;
  using Bvh = bvh::Bvh<Scalar>;

  auto os = OriginSphere<T>(state.range(0));

  std::vector<Triangle> triangles;
  triangles.reserve(os.triangles.size());
  for (int i = 0; i < os.triangles.size(); i += 3) {
    triangles.emplace_back(
        Vector3{
            static_cast<Scalar>(os.triangles[i][0]),
            static_cast<Scalar>(os.triangles[i][1]),
            static_cast<Scalar>(os.triangles[i][2])},
        Vector3{
            static_cast<Scalar>(os.triangles[i + 1][0]),
            static_cast<Scalar>(os.triangles[i + 1][1]),
            static_cast<Scalar>(os.triangles[i + 1][2])},
        Vector3{
            static_cast<Scalar>(os.triangles[i + 2][0]),
            static_cast<Scalar>(os.triangles[i + 2][1]),
            static_cast<Scalar>(os.triangles[i + 2][2])});
  }

  Bvh bvh;

  // Create an acceleration data structure on those triangles
  bvh::SweepSahBuilder<Bvh> builder(bvh);
  auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(triangles.data(), triangles.size());
  auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), triangles.size());

  for (auto _ : state) {
    builder.build(global_bbox, bboxes.get(), centers.get(), triangles.size());
  }
}
BENCHMARK_TEMPLATE(BM_bvh_BUILD_BHV_Sphere_SweepSAH, float)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(BM_bvh_BUILD_BHV_Sphere_SweepSAH, double)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);

template<typename T>
static void BM_bvh_BUILD_BHV_Sphere_BinnedSAH(benchmark::State &state) {
  using Scalar = T;
  using Vector3 = bvh::Vector3<Scalar>;
  using Triangle = bvh::Triangle<Scalar>;
  using Ray = bvh::Ray<Scalar>;
  using Bvh = bvh::Bvh<Scalar>;

  auto os = OriginSphere<T>(state.range(0));

  std::vector<Triangle> triangles;
  triangles.reserve(os.triangles.size());
  for (int i = 0; i < os.triangles.size(); i += 3) {
    triangles.emplace_back(
        Vector3{
            static_cast<Scalar>(os.triangles[i][0]),
            static_cast<Scalar>(os.triangles[i][1]),
            static_cast<Scalar>(os.triangles[i][2])},
        Vector3{
            static_cast<Scalar>(os.triangles[i + 1][0]),
            static_cast<Scalar>(os.triangles[i + 1][1]),
            static_cast<Scalar>(os.triangles[i + 1][2])},
        Vector3{
            static_cast<Scalar>(os.triangles[i + 2][0]),
            static_cast<Scalar>(os.triangles[i + 2][1]),
            static_cast<Scalar>(os.triangles[i + 2][2])});
  }

  Bvh bvh;

  // Create an acceleration data structure on those triangles
  bvh::BinnedSahBuilder<Bvh, 64> builder(bvh);
  auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(triangles.data(), triangles.size());
  auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), triangles.size());

  for (auto _ : state) {
    builder.build(global_bbox, bboxes.get(), centers.get(), triangles.size());
  }
}
BENCHMARK_TEMPLATE(BM_bvh_BUILD_BHV_Sphere_BinnedSAH, float)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(BM_bvh_BUILD_BHV_Sphere_BinnedSAH, double)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);