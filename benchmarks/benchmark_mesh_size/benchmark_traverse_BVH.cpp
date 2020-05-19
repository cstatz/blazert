//
// Created by ogarten on 19/05/2020.
//
//#define BLAZERT_USE_PADDED_AND_ALIGNED_TYPES
#include <benchmark/benchmark.h>
#include "../benchmark_helper/OriginSphere.h"

#include <blazert/blazert.h>
#include <blazert/primitives/trimesh.h>

#include <embree3/rtcore.h>

using namespace blazert;

template<typename T>
static void BM_BLAZERT_TRAVERSE_BVH_OriginSphere(benchmark::State& state)
{
  BVHBuildOptions<T> build_options;
  BVHTraceOptions<T> trace_options;

  auto os = OriginSphere<T>(state.range(0));

  TriangleMesh triangles(os.vertices, os.triangles);
  TriangleSAHPred triangles_sah(os.vertices, os.triangles);

  BVH<T> triangles_bvh;
  const bool success = triangles_bvh.build(triangles, triangles_sah, build_options);
  //std::cout << "success = " << success << "\n";
  const Vec3r<T> org{0,0.01,10};
  const Vec3r<T> dir{0,0,-1};
  const Ray<T> ray{org, dir};
  for (auto _ : state) {
    TriangleIntersector<T> triangle_intersector{os.vertices, os.triangles};
    RayHit<T> temp_rayhit;
    traverse(triangles_bvh, ray, triangle_intersector, temp_rayhit, trace_options);
  }
}
BENCHMARK_TEMPLATE(BM_BLAZERT_TRAVERSE_BVH_OriginSphere, float)->DenseRange(2,8,1);//->Unit(benchmark::kMillisecond);
BENCHMARK_TEMPLATE(BM_BLAZERT_TRAVERSE_BVH_OriginSphere, double)->DenseRange(2,8,1);

static void
BM_EMBREE_TRAVERSE_BVH_OriginSphere(benchmark::State& state)
{
  auto device = rtcNewDevice("verbose=0,start_threads=1,threads=1,set_affinity=1");
  auto scene =  rtcNewScene(device);

  constexpr const int bytestride_int = sizeof(Vec3ui) / 8 * sizeof(Vec3ui::ElementType);
  constexpr const int bytestride_float = sizeof(Vec3r<float>) / 8 * sizeof(Vec3r<float>::ElementType);

  const auto os = OriginSphere<float>(state.range(0));

  auto geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetSharedGeometryBuffer(geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, (void *) (os.triangles.data()), 0, bytestride_int, os.triangles.size());
  rtcSetSharedGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, (void *) (os.vertices.data()), 0, bytestride_float, os.vertices.size());
  rtcCommitGeometry(geometry);
  rtcAttachGeometry(scene, geometry);
  rtcCommitScene(scene);

  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  const Vec3r<float> org{0,0.01,10};
  const Vec3r<float> dir{0,0,-1};

  const RTCRay r{org[0], org[1], org[2], 0, dir[0], dir[1], dir[2], 0, std::numeric_limits<float>::max(), 0, 0, 0};

  for (auto _ : state) {
    RTCHit h{};
    RTCRayHit rh{r, h};
    rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
    rtcIntersect1(scene, &context, &rh);
  }
}
BENCHMARK(BM_EMBREE_TRAVERSE_BVH_OriginSphere)->DenseRange(2,8,1);// ->Unit(benchmark::kMillisecond);