//
// Created by ogarten on 12/05/2020.
//
//
// Created by ogarten on 1/23/19.
//

#include <blazert/blazert.h>
#include <blazert/bvh/accel.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/sphere.h>
#include <blazert/ray.h>
#include <memory>
//#include <blazert/scene.h>

#include "../catch.hpp"
#include "../test_helpers.h"

using namespace blazert;

TEMPLATE_TEST_CASE("Sphere", "[bounding box, distance to surface, intersections]", float, double) {

  SECTION("BOUNDING BOX") {
    const Vec3r<TestType> center{1.f, 1.f, 1.f};
    float radius = 1.f;

    auto centers = std::make_unique<Vec3rList<TestType>>();
    auto radiuss = std::make_unique<std::vector<TestType>>();
    centers->emplace_back(center);
    radiuss->emplace_back(radius);

    Vec3r<TestType> bmin, bmax;
    Sphere<TestType> sphere(*centers, *radiuss);
    sphere.BoundingBox(bmin, bmax, 0);

    REQUIRE(bmin[0] == Approx(0.f));
    REQUIRE(bmin[1] == Approx(0.f));
    REQUIRE(bmin[2] == Approx(0.f));
    REQUIRE(bmax[0] == Approx(2.f));
    REQUIRE(bmax[1] == Approx(2.f));
    REQUIRE(bmax[2] == Approx(2.f));

  }

  //  SECTION("DISTANCE TO SURFACE") {
  //    Vec3r<TestType> center{0.f, 0.f, 0.f};
  //    SECTION("R = 1") {
  //      float radius = 1.f;
  //
  //      EmbreeSphere sphere(D, S, center, radius);
  //      rtcCommitScene(S);
  //
  //      REQUIRE(sphere.distance_to_surface(Vec3r<TestType>{0.f, 0.f, 0.f}) == Approx(1.f));
  //      REQUIRE(sphere.distance_to_surface(Vec3r<TestType>{-1.f, 0.f, 0.f}) == Approx(0.f));
  //      REQUIRE(sphere.distance_to_surface(Vec3r<TestType>{-2.f, 0.f, 0.f}) == Approx(1.f));
  //      REQUIRE(sphere.distance_to_surface(Vec3r<TestType>{-2.f, -2.f, -2.f}) == Approx(std::sqrt(3.f * 4.f) - 1.f));
  //    }
  //    SECTION("R = 3") {
  //      float radius = 3.f;
  //
  //      //EmbreeDevice D("verbose=0");
  //      //EmbreeScene S(D, RTC_SCENE_FLAG_NONE, 0);
  //      EmbreeSphere sphere(D, S, center, radius);
  //      rtcCommitScene(S);
  //
  //      REQUIRE(sphere.distance_to_surface(Vec3r<TestType>{0.f, 0.f, 0.f}) == Approx(3.f));
  //      REQUIRE(sphere.distance_to_surface(Vec3r<TestType>{-1.f, 0.f, 0.f}) == Approx(2.f));
  //      REQUIRE(sphere.distance_to_surface(Vec3r<TestType>{-2.f, 0.f, 0.f}) == Approx(1.f));
  //      REQUIRE(sphere.distance_to_surface(Vec3r<TestType>{-2.f, -2.f, -2.f}) == Approx(std::sqrt(3.f * 4.f) - 3.f));
  //    }
  //  }
  SECTION("INTERSECTIONS") {
    SECTION("Ray origin outside sphere") {
      Vec3r<TestType> center{0.f, 0.f, 0.f};
      TestType radius = 1.;

      Vec3r<TestType> org{2.f, 0.f, 0.f};
      Vec3r<TestType> dir{-1.f, 0.f, 0.f};

      Ray<TestType> ray{org, dir};
      RayHit<TestType> rayhit;

      // Centers and Radii should go on the heap.
    auto centers = std::make_unique<Vec3rList<TestType>>();
    auto radiuss = std::make_unique<std::vector<TestType>>();

      centers->emplace_back(center);
      radiuss->emplace_back(radius);

      Sphere<TestType> sphere(*centers, *radiuss);
      SphereSAHPred<TestType> sphere_sah(*centers, *radiuss);

      BVHBuildOptions<TestType> build_options;
      BVHTraceOptions<TestType> trace_options;

      BVH<TestType> bvh_sphere;
      bvh_sphere.build(sphere, sphere_sah, build_options);

      SphereIntersector<TestType> sphere_intersector{*centers, *radiuss};
      const bool hit_sphere = traverse(bvh_sphere, ray, sphere_intersector, rayhit, trace_options);

      // should be in distance of 1
      REQUIRE(hit_sphere);
      REQUIRE(rayhit.hit_distance == Approx(static_cast<TestType>(1.f)));
    }
  }
}

TEMPLATE_TEST_CASE("Scene with Sphere", "[]", float, double) {
  SECTION("Intersection") {
    auto centers = std::make_unique<Vec3rList<TestType>>();
    auto radiuss = std::make_unique<std::vector<TestType>>();

    centers->emplace_back(Vec3r<TestType>{0.});
    radiuss->emplace_back(1.);

    Vec3r<TestType> org{2.f, 0.f, 0.f};
    Vec3r<TestType> dir{-1.f, 0.f, 0.f};

    Scene<TestType> scene;
    unsigned int prim_id = scene.add_spheres(*centers, *radiuss);
    scene.commit();

    const Ray<TestType> ray{org, dir};
    RayHit<TestType> rayhit;
    rayhit.hit_distance = std::numeric_limits<TestType>::max();

    const bool hit = intersect1(scene, ray, rayhit);

    REQUIRE(prim_id == 0);
    REQUIRE(hit);
    REQUIRE(rayhit.hit_distance == Approx(1));
  }
}