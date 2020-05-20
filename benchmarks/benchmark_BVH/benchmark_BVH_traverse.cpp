//
// Created by ogarten on 19/05/2020.
//
//#define BLAZERT_USE_PADDED_AND_ALIGNED_TYPES
#include "../benchmark_helper/OriginSphere.h"
#include <benchmark/benchmark.h>

#include <blazert/blazert.h>
#include <blazert/primitives/trimesh.h>

#include <third_party/nanort/nanort.h>

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
static void BM_BLAZERT_TRAVERSE_BVH_Sphere(benchmark::State &state) {
  BVHBuildOptions<T> build_options;
  BVHTraceOptions<T> trace_options;

  auto os = OriginSphere<T>(state.range(0));

  TriangleMesh triangles(os.vertices, os.triangles);
  TriangleSAHPred triangles_sah(os.vertices, os.triangles);

  BVH<T> triangles_bvh;
  const bool success = triangles_bvh.build(triangles, triangles_sah, build_options);
  //std::cout << "success = " << success << "\n";

  constexpr int height = 4 * 2048;
  constexpr int width = 4 * 2048;
  for (auto _ : state) {
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        const blazert::Ray<T> ray{{0.0, 0.0, 10.0}, {static_cast<T>((x / T(width)) - 0.5), static_cast<T>((y / T(height)) - 0.5), T(-1.)}};
        TriangleIntersector<T> triangle_intersector{os.vertices, os.triangles};
        RayHit<T> temp_rayhit;
        traverse(triangles_bvh, ray, triangle_intersector, temp_rayhit, trace_options);
      }
    }
  }
}
BENCHMARK_TEMPLATE(BM_BLAZERT_TRAVERSE_BVH_Sphere, float)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(BM_BLAZERT_TRAVERSE_BVH_Sphere, double)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);

static void
BM_EMBREE_TRAVERSE_BVH_Sphere(benchmark::State &state) {
  auto device = rtcNewDevice("verbose=0,start_threads=1,threads=1,set_affinity=1");
  auto scene = rtcNewScene(device);

  constexpr const int bytestride_int = sizeof(Vec3ui) / 4 * sizeof(Vec3ui::ElementType);
  constexpr const int bytestride_float = sizeof(Vec3r<float>) / 4 * sizeof(Vec3r<float>::ElementType);

  const auto os = OriginSphere<float>(state.range(0));

  auto geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetSharedGeometryBuffer(geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, (void *) (os.triangles.data()), 0, bytestride_int, os.triangles.size());
  rtcSetSharedGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, (void *) (os.vertices.data()), 0, bytestride_float, os.vertices.size());
  rtcCommitGeometry(geometry);
  rtcAttachGeometry(scene, geometry);
  rtcCommitScene(scene);

  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  const Vec3r<float> org{0, 0, 10};

  constexpr int height = 4 * 2048;
  constexpr int width = 4 * 2048;
  for (auto _ : state) {
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        const RTCRay r{org[0], org[1], org[2],
                       0,
                       static_cast<float>((x / float(width)) - 0.5), static_cast<float>((y / float(height)) - 0.5), float(-1.),
                       0, std::numeric_limits<float>::max(), 0, 0, 0};
        RTCHit h{};
        RTCRayHit rh{r, h};
        rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
        rtcIntersect1(scene, &context, &rh);
      }
    }
  }
}
BENCHMARK(BM_EMBREE_TRAVERSE_BVH_Sphere)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);

template<typename T>
static void BM_nanoRT_TRAVERSE_BVH_Sphere(benchmark::State &state) {
  nanort::BVHBuildOptions<T> build_options;
  nanort::BVHTraceOptions trace_options;

  auto os = OriginSphere<T>(state.range(0));

  const Vec3r<T> *verts = os.vertices.data();
  const Vec3ui *tris = os.triangles.data();

  nanort::TriangleMesh<T> triangles{verts->data(), tris->data(), sizeof(Vec3r<T>)};
  nanort::TriangleSAHPred<T> triangles_sah{verts->data(), tris->data(), sizeof(Vec3r<T>)};

  nanort::BVHAccel<T> triangles_bvh;
  const bool success = triangles_bvh.Build(os.triangle_count(), triangles, triangles_sah, build_options);
  //std::cout << "success = " << success << "\n";

  constexpr int height = 4 * 2048;
  constexpr int width = 4 * 2048;
  for (auto _ : state) {
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        nanort::Ray<T> ray;
        ray.min_t = 0.0f;
        ray.max_t = std::numeric_limits<T>::max();
        ray.org[0] = 0.0f;
        ray.org[1] = 5.0f;
        ray.org[2] = 20.0f;
        nanort::real3 dir;
        dir[0] = (x / (T) width) - 0.5f;
        dir[1] = (y / (T) height) - 0.5f;
        dir[2] = -1.0f;
        dir = nanort::vnormalize(dir);
        ray.dir[0] = dir[0];
        ray.dir[1] = dir[1];
        ray.dir[2] = dir[2];
        nanort::TriangleIntersector<T> triangle_intersector{verts->data(), tris->data(), sizeof(Vec3r<T>)};
        nanort::TriangleIntersection<T> temp_rayhit;
        triangles_bvh.Traverse(ray, triangle_intersector, &temp_rayhit, trace_options);
      }
    }
  }
}
//BENCHMARK_TEMPLATE(BM_nanoRT_TRAVERSE_BVH_Sphere, float)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);
//BENCHMARK_TEMPLATE(BM_nanoRT_TRAVERSE_BVH_Sphere, double)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);

template<typename T>
static void BM_bvh_TRAVERSE_BVH_Sphere(benchmark::State &state) {
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
  builder.build(global_bbox, bboxes.get(), centers.get(), triangles.size());

  constexpr int height = 4 * 2048;
  constexpr int width = 4 * 2048;
  for (auto _ : state) {
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        // Intersect a ray with the data structure
        Ray ray(
            Vector3(0.0, 0.0, 1.0),                                                                     // origin
            Vector3(static_cast<T>((x / T(width)) - 0.5), static_cast<T>((y / T(height)) - 0.5), T(-1)),// direction
            0.0,                                                                                        // minimum distance
            std::numeric_limits<T>::max()                                                               // maximum distance
        );
        bvh::ClosestIntersector<false, Bvh, Triangle> intersector(bvh, triangles.data());
        bvh::SingleRayTraverser<Bvh> traverser(bvh);

        auto hit = traverser.traverse(ray, intersector);
      }
    }
  }
}
BENCHMARK_TEMPLATE(BM_bvh_TRAVERSE_BVH_Sphere, float)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(BM_bvh_TRAVERSE_BVH_Sphere, double)->DenseRange(2, 9, 1)->Unit(benchmark::kMillisecond);