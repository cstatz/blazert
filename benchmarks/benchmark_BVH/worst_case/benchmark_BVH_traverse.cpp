//
// Created by ogarten on 19/05/2020.
//
//#define BLAZERT_USE_PADDED_AND_ALIGNED_TYPES
#include "../../benchmark_helper/OriginSphere.h"
#include <benchmark/benchmark.h>

#include <blazert/blazert.h>
#include <blazert/primitives/trimesh.h>

#include <third_party/nanort/nanort.h>

#include <third_party/bvh/include/bvh/binned_sah_builder.hpp>
#include <third_party/bvh/include/bvh/bvh.hpp>
#include <third_party/bvh/include/bvh/intersectors.hpp>
#include <third_party/bvh/include/bvh/ray.hpp>
#include <third_party/bvh/include/bvh/single_ray_traverser.hpp>
#include <third_party/bvh/include/bvh/sweep_sah_builder.hpp>
#include <third_party/bvh/include/bvh/triangle.hpp>
#include <third_party/bvh/include/bvh/vector.hpp>

#include <embree3/rtcore.h>

using namespace blazert;

template<typename T>
static void BM_BLAZERT_TRAVERSE_WORST_Sphere(benchmark::State &state) {
  BVHBuildOptions<T> build_options;

  const auto os = std::make_unique<OriginSphere<T>>(state.range(0));

  TriangleMesh triangles(os->vertices, os->triangles);

  BVH triangles_bvh(triangles);
  SAHBinnedBuilder builder;
  [[maybe_unused]] auto stats = builder.build(triangles_bvh, build_options);
  //std::cout << "success = " << success << "\n";

  for (auto _ : state) {
    for (auto &dir : os->vertices) {
      const blazert::Ray<T> ray{{0.0, 0.0, 0.0}, dir};
      RayHit<T> temp_rayhit;
      const auto hit = traverse(triangles_bvh, ray, temp_rayhit);
      benchmark::DoNotOptimize(hit);
    }
  }
}
BENCHMARK_TEMPLATE(BM_BLAZERT_TRAVERSE_WORST_Sphere, float)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(BM_BLAZERT_TRAVERSE_WORST_Sphere, double)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);

#ifdef EMBREE_TRACING
static void
BM_EMBREE_TRAVERSE_WORST_Sphere(benchmark::State &state) {
  using embreeVec3 = blaze::StaticVector<float, 3UL, blaze::columnVector, blaze::AlignmentFlag::aligned, blaze::PaddingFlag::padded>;
  using embreeVec3List = std::vector<embreeVec3, blaze::AlignedAllocator<embreeVec3>>;

  using embreeVec3ui = blaze::StaticVector<unsigned int, 3UL, blaze::columnVector, blaze::AlignmentFlag::aligned, blaze::PaddingFlag::padded>;
  using embreeVec3iList = std::vector<embreeVec3ui,  blaze::AlignedAllocator<embreeVec3ui>>;

  auto device = rtcNewDevice("verbose=0,start_threads=1,threads=1,set_affinity=1");
  auto scene = rtcNewScene(device);

  constexpr const int bytestride_int = sizeof(embreeVec3ui) / 8 * sizeof(embreeVec3ui::ElementType);
  constexpr const int bytestride_float = sizeof(embreeVec3) / 8 * sizeof(embreeVec3::ElementType);

  const auto os = std::make_unique<OriginSphere<float>>(state.range(0));

  auto vertices = std::make_unique<embreeVec3List>();
  vertices->reserve(os->vertices.size());
  auto triangles = std::make_unique<embreeVec3iList>(os->triangles.size());
  triangles->reserve(os->triangles.size());

  for(auto &v : os->vertices) {
    vertices->emplace_back(embreeVec3{v[0], v[1], v[2]});
  }
  for(auto &t : os->triangles) {
    triangles->emplace_back(embreeVec3ui{t[0], t[1], t[2]});
  }

  auto geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetSharedGeometryBuffer(geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, (void *) (triangles->data()), 0, bytestride_int, triangles->size());
  rtcSetSharedGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, (void *) (vertices->data()), 0, bytestride_float, vertices->size());
  rtcCommitGeometry(geometry);
  rtcAttachGeometry(scene, geometry);
  rtcCommitScene(scene);

  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  const Vec3r<float> org{0, 0, 0};

  for (auto _ : state) {
    for (auto &dir : os->vertices) {
      const RTCRay r{org[0], org[1], org[2],
                     0,
                     dir[0], dir[1], dir[2],
                     0, std::numeric_limits<float>::max(), 0, 0, 0};
      RTCHit h{};
      RTCRayHit rh{r, h};
      rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
      rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
      rtcIntersect1(scene, &context, &rh);
      benchmark::DoNotOptimize(rh);
    }
  }
}
BENCHMARK(BM_EMBREE_TRAVERSE_WORST_Sphere)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);
#endif

template<typename T>
static void BM_nanoRT_TRAVERSE_WORST_Sphere(benchmark::State &state) {
  nanort::BVHBuildOptions<T> build_options;
  nanort::BVHTraceOptions trace_options;

  const auto os = std::make_unique<OriginSphere<T>>(state.range(0));

  const Vec3r<T> *verts = os->vertices.data();
  const Vec3ui *tris = os->triangles.data();

  nanort::TriangleMesh<T> triangles{verts->data(), tris->data(), sizeof(Vec3r<T>)};
  nanort::TriangleSAHPred<T> triangles_sah{verts->data(), tris->data(), sizeof(Vec3r<T>)};

  nanort::BVHAccel<T> triangles_bvh;
  [[maybe_unused]] const bool success = triangles_bvh.Build(os->triangle_count(), triangles, triangles_sah, build_options);
  //std::cout << "success = " << success << "\n";

  for (auto _ : state) {
    for (auto &dir : os->vertices) {
      nanort::Ray<T> ray;
      ray.min_t = 0.0f;
      ray.max_t = std::numeric_limits<T>::max();
      ray.org[0] = 0.0f;
      ray.org[1] = 0.0f;
      ray.org[2] = 0.0f;
      nanort::real3 nanort_dir;
      nanort_dir[0] = dir[0];
      nanort_dir[1] = dir[1];
      nanort_dir[2] = dir[2];
      nanort_dir = nanort::vnormalize(nanort_dir);
      ray.dir[0] = nanort_dir[0];
      ray.dir[1] = nanort_dir[1];
      ray.dir[2] = nanort_dir[2];
      nanort::TriangleIntersector<T> triangle_intersector{verts->data(), tris->data(), sizeof(Vec3r<T>)};
      nanort::TriangleIntersection<T> temp_rayhit;
      const auto hit = triangles_bvh.Traverse(ray, triangle_intersector, &temp_rayhit, trace_options);
      benchmark::DoNotOptimize(hit);
    }
  }
}
BENCHMARK_TEMPLATE(BM_nanoRT_TRAVERSE_WORST_Sphere, float)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(BM_nanoRT_TRAVERSE_WORST_Sphere, double)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);

template<typename T>
static void BM_bvh_TRAVERSE_WORST_Sphere_SweepSAH(benchmark::State &state) {
  using Scalar = T;
  using Vector3 = bvh::Vector3<Scalar>;
  using Triangle = bvh::Triangle<Scalar>;
  using Ray = bvh::Ray<Scalar>;
  using Bvh = bvh::Bvh<Scalar>;

  const auto os = std::make_unique<OriginSphere<T>>(state.range(0));

  std::vector<Triangle> triangles;
  triangles.reserve(os->triangles.size());
  for (uint32_t i = 0; i < os->triangles.size(); i += 3) {
    triangles.emplace_back(
        Vector3{
            static_cast<Scalar>(os->triangles[i][0]),
            static_cast<Scalar>(os->triangles[i][1]),
            static_cast<Scalar>(os->triangles[i][2])},
        Vector3{
            static_cast<Scalar>(os->triangles[i + 1][0]),
            static_cast<Scalar>(os->triangles[i + 1][1]),
            static_cast<Scalar>(os->triangles[i + 1][2])},
        Vector3{
            static_cast<Scalar>(os->triangles[i + 2][0]),
            static_cast<Scalar>(os->triangles[i + 2][1]),
            static_cast<Scalar>(os->triangles[i + 2][2])});
  }

  Bvh bvh;

  // Create an acceleration data structure on those triangles
  bvh::SweepSahBuilder<Bvh> builder(bvh);
  auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(triangles.data(), triangles.size());
  auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), triangles.size());
  builder.build(global_bbox, bboxes.get(), centers.get(), triangles.size());

  for (auto _ : state) {
    for (auto &dir : os->vertices) {
      // Intersect a ray with the data structure
      Ray ray(
          Vector3(0.0, 0.0, 0.0),         // origin
          Vector3(dir[0], dir[1], dir[2]),// direction
          0.0,                            // minimum distance
          std::numeric_limits<T>::max()   // maximum distance
      );
      bvh::ClosestIntersector<false, Bvh, Triangle> intersector(bvh, triangles.data());
      bvh::SingleRayTraverser<Bvh> traverser(bvh);

      const auto hit = traverser.traverse(ray, intersector);
      benchmark::DoNotOptimize(hit);
    }
  }
}
BENCHMARK_TEMPLATE(BM_bvh_TRAVERSE_WORST_Sphere_SweepSAH, float)->DenseRange(2, 8, 1)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(BM_bvh_TRAVERSE_WORST_Sphere_SweepSAH, double)->DenseRange(2, 8, 1)->Unit(benchmark::kMillisecond);

template<typename T>
static void BM_bvh_TRAVERSE_WORST_Sphere_BinnedSAH(benchmark::State &state) {
  using Scalar = T;
  using Vector3 = bvh::Vector3<Scalar>;
  using Triangle = bvh::Triangle<Scalar>;
  using Ray = bvh::Ray<Scalar>;
  using Bvh = bvh::Bvh<Scalar>;

  const auto os = std::make_unique<OriginSphere<T>>(state.range(0));

  std::vector<Triangle> triangles;
  triangles.reserve(os->triangles.size());
  for (uint32_t i = 0; i < os->triangles.size(); i += 3) {
    triangles.emplace_back(
        Vector3{
            static_cast<Scalar>(os->triangles[i][0]),
            static_cast<Scalar>(os->triangles[i][1]),
            static_cast<Scalar>(os->triangles[i][2])},
        Vector3{
            static_cast<Scalar>(os->triangles[i + 1][0]),
            static_cast<Scalar>(os->triangles[i + 1][1]),
            static_cast<Scalar>(os->triangles[i + 1][2])},
        Vector3{
            static_cast<Scalar>(os->triangles[i + 2][0]),
            static_cast<Scalar>(os->triangles[i + 2][1]),
            static_cast<Scalar>(os->triangles[i + 2][2])});
  }

  Bvh bvh;

  // Create an acceleration data structure on those triangles
  bvh::BinnedSahBuilder<Bvh, 64> builder(bvh);
  auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(triangles.data(), triangles.size());
  auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), triangles.size());
  builder.build(global_bbox, bboxes.get(), centers.get(), triangles.size());

  for (auto _ : state) {
    for (auto &dir : os->vertices) {
      // Intersect a ray with the data structure
      Ray ray(
          Vector3(0.0, 0.0, 0.0),         // origin
          Vector3(dir[0], dir[1], dir[2]),// direction
          0.0,                            // minimum distance
          std::numeric_limits<T>::max()   // maximum distance
      );
      bvh::ClosestIntersector<false, Bvh, Triangle> intersector(bvh, triangles.data());
      bvh::SingleRayTraverser<Bvh> traverser(bvh);

      const auto hit = traverser.traverse(ray, intersector);
      benchmark::DoNotOptimize(hit);
    }
  }
}
BENCHMARK_TEMPLATE(BM_bvh_TRAVERSE_WORST_Sphere_BinnedSAH, float)->DenseRange(2, 8, 1)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(BM_bvh_TRAVERSE_WORST_Sphere_BinnedSAH, double)->DenseRange(2, 8, 1)->Unit(benchmark::kMillisecond);
