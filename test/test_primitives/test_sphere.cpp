//
// Created by ogarten on 12/05/2020.
//
/*
#include <blazert/blazert.h>
#include <blazert/bvh/accel.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/sphere.h>
#include <blazert/ray.h>
#include <memory>
//#include <blazert/scene.h>

#include <third_party/doctest/doctest/doctest.h>
#include "../test_helpers.h"

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("Sphere", T, float, double) {

  SUBCASE("bounding box") {
    const Vec3r<T> center{1.f, 1.f, 1.f};
    T radius = 1.f;

    auto centers = std::make_unique<Vec3rList<T>>();
    auto radii = std::make_unique<std::vector<T>>();
    centers->emplace_back(center);
    radii->emplace_back(radius);

    Sphere<T> sphere(*centers, *radii);

    Vec3r<T> bmin, bmax;
    sphere.BoundingBox(bmin, bmax, 0);

    CHECK(bmin[0] == Approx(0.f));
    CHECK(bmin[1] == Approx(0.f));
    CHECK(bmin[2] == Approx(0.f));
    CHECK(bmax[0] == Approx(2.f));
    CHECK(bmax[1] == Approx(2.f));
    CHECK(bmax[2] == Approx(2.f));
  }
  SUBCASE("distance to surface") {
    Vec3r<T> center{0.f, 0.f, 0.f};
    auto centers = std::make_unique<Vec3rList<T>>();
    auto radii = std::make_unique<std::vector<T>>();
    centers->emplace_back(center);
    
    SUBCASE("R = 1")
    {
      float radius = 1.f;
      radii->emplace_back(radius);
      
      Sphere<T> spheres(*centers, *radii);

      CHECK(distance_to_surface(spheres, Vec3r<T>{  0.f,  0.f,  0.f }, 0) == Approx(1.f));
      CHECK(distance_to_surface(spheres, Vec3r<T>{ -1.f,  0.f,  0.f }, 0) == Approx(0.f));
      CHECK(distance_to_surface(spheres, Vec3r<T>{ -2.f,  0.f,  0.f }, 0) == Approx(1.f));
      CHECK(distance_to_surface(spheres, Vec3r<T>{ -2.f, -2.f, -2.f }, 0) == Approx(std::sqrt(3.f * 4.f) - 1.f));
    }
    SUBCASE("R = 3")
    {
      float radius = 3.f;
      radii->emplace_back(radius);

      Sphere<T> spheres(*centers, *radii);

      CHECK(distance_to_surface(spheres, Vec3r<T>{  0.f,  0.f,  0.f }, 0) == Approx(3.f));
      CHECK(distance_to_surface(spheres, Vec3r<T>{ -1.f,  0.f,  0.f }, 0) == Approx(2.f));
      CHECK(distance_to_surface(spheres, Vec3r<T>{ -2.f,  0.f,  0.f }, 0) == Approx(1.f));
      CHECK(distance_to_surface(spheres, Vec3r<T>{ -2.f, -2.f, -2.f }, 0) == Approx(std::sqrt(3.f * 4.f) - 3.f));
    }
  }
  SUBCASE("intersections") {
    SUBCASE("Ray origin outside sphere") {
      const Vec3r<T> center{0.f, 0.f, 0.f};
      const T radius = 1.;

      // Centers and Radii should go on the heap.
      auto centers = std::make_unique<Vec3rList<T>>();
      auto radii = std::make_unique<std::vector<T>>();
      centers->emplace_back(center);
      radii->emplace_back(radius);

      const Vec3r<T> org{2.f, 0.f, 0.f};
      const Vec3r<T> dir{-1.f, 0.f, 0.f};

      const Ray<T> ray{org, dir};
      RayHit<T> rayhit;

      BVHTraceOptions<T> trace_options;

      SphereIntersector<T> sphere_intersector{*centers, *radii};

      // Test intersections
      update_intersector(sphere_intersector, ray.max_hit_distance, -1);
      prepare_traversal(sphere_intersector, ray, trace_options);
      T hit_distance = sphere_intersector.hit_distance;
      const bool hit_sphere = intersect(sphere_intersector, hit_distance, 0);
      update_intersector(sphere_intersector, hit_distance, 0);
      post_traversal(sphere_intersector, ray, hit_sphere, rayhit);

      // should be in distance of 1
      CHECK(hit_sphere);
      CHECK(rayhit.hit_distance == Approx(static_cast<T>(1.f)));
    }
    SUBCASE("Ray origin inside sphere") {
      Vec3r<T> center1{0.f, 0.f, 0.f};
      T radius = 1.f;

      auto centers = std::make_unique<Vec3rList<T>>();
      auto radii = std::make_unique<std::vector<T>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      SUBCASE("Ray 1") {
        const Vec3r<T> org{0.f, 0.f, 0.f};
        const Vec3r<T> dir{-1.f, 0.f, 0.f};

        const Ray<T> ray{org, dir};
        RayHit<T> rayhit;

        BVHTraceOptions<T> trace_options;

        SphereIntersector<T> sphere_intersector{*centers, *radii};

        // Test intersections
        update_intersector(sphere_intersector, ray.max_hit_distance, -1);
        prepare_traversal(sphere_intersector, ray, trace_options);
        T hit_distance = sphere_intersector.hit_distance;
        const bool hit_sphere = intersect(sphere_intersector, hit_distance, 0);
        update_intersector(sphere_intersector, hit_distance, 0);
        post_traversal(sphere_intersector, ray, hit_sphere, rayhit);

        // should be in distance of 1
        CHECK(hit_sphere);
        CHECK(rayhit.hit_distance == Approx(1.f));
      }
      SUBCASE("Ray 2") {
        const Vec3r<T> org{0.f, 0.f, 0.5f};
        const Vec3r<T> dir{0.f, 0.f, -1.f};

        const Ray<T> ray{org, dir};
        RayHit<T> rayhit;

        BVHTraceOptions<T> trace_options;

        SphereIntersector<T> sphere_intersector{*centers, *radii};

        // Test intersections
        update_intersector(sphere_intersector, ray.max_hit_distance, -1);
        prepare_traversal(sphere_intersector, ray, trace_options);
        T hit_distance = sphere_intersector.hit_distance;
        const bool hit_sphere = intersect(sphere_intersector, hit_distance, 0);
        update_intersector(sphere_intersector, hit_distance, 0);
        post_traversal(sphere_intersector, ray, hit_sphere, rayhit);

        // should be in distance of 1
        CHECK(hit_sphere);
        CHECK(rayhit.hit_distance == Approx(1.5f));
      }
    }
    SUBCASE("Ray origin on sphere") {
      Vec3r<T> center1{1.f, 0.f, 0.f};
      T radius = 1.f;

      auto centers = std::make_unique<Vec3rList<T>>();
      auto radii = std::make_unique<std::vector<T>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      SUBCASE("Ray 1") {
        Vec3r<T> org{0.f, 0.f, 0.f};
        Vec3r<T> dir{1.f, 0.f, 0.f};

        const Ray<T> ray{org, dir};
        RayHit<T> rayhit;

        BVHTraceOptions<T> trace_options;

        SphereIntersector<T> sphere_intersector{*centers, *radii};

        // Test intersections
        update_intersector(sphere_intersector, ray.max_hit_distance, -1);
        prepare_traversal(sphere_intersector, ray, trace_options);
        T hit_distance = sphere_intersector.hit_distance;
        const bool hit_sphere = intersect(sphere_intersector, hit_distance, 0);
        update_intersector(sphere_intersector, hit_distance, 0);
        post_traversal(sphere_intersector, ray, hit_sphere, rayhit);

        CHECK(rayhit.hit_distance == Approx(2.f));
      }
      SUBCASE("Ray 2") {
        Vec3r<T> org{0.f, 0.f, 0.f};
        Vec3r<T> dir{-1.f, 0.f, 0.f};

        const Ray<T> ray{org, dir};
        RayHit<T> rayhit;

        BVHTraceOptions<T> trace_options;

        SphereIntersector<T> sphere_intersector{*centers, *radii};

        // Test intersections
        update_intersector(sphere_intersector, ray.max_hit_distance, -1);
        prepare_traversal(sphere_intersector, ray, trace_options);
        T hit_distance = sphere_intersector.hit_distance;
        const bool hit_sphere = intersect(sphere_intersector, hit_distance, 0);
        update_intersector(sphere_intersector, hit_distance, 0);
        post_traversal(sphere_intersector, ray, hit_sphere, rayhit);

        CHECK(rayhit.hit_distance == Approx(std::numeric_limits<T>::max()));
      }
    }
    SUBCASE("ray passing through sphere center") {
      Vec3r<T> center1{0.f, 0.f, 0.f};
      T radius = 2.f;

      auto centers = std::make_unique<Vec3rList<T>>();
      auto radii = std::make_unique<std::vector<T>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      Vec3r<T> org{3.f, 1.f, 0.f};
      Vec3r<T> dir{-1.f, 0.f, 0.f};

      const Ray<T> ray{org, dir};
      RayHit<T> rayhit;

      BVHTraceOptions<T> trace_options;

      SphereIntersector<T> sphere_intersector{*centers, *radii};

      // Test intersections
      update_intersector(sphere_intersector, ray.max_hit_distance, -1);
      prepare_traversal(sphere_intersector, ray, trace_options);
      T hit_distance = sphere_intersector.hit_distance;
      const bool hit_sphere = intersect(sphere_intersector, hit_distance, 0);
      update_intersector(sphere_intersector, hit_distance, 0);
      post_traversal(sphere_intersector, ray, hit_sphere, rayhit);

      CHECK(rayhit.hit_distance == Approx(1.26794919f));

      CHECK(rayhit.normal[0] == Approx(sqrt(3) / 2));
      CHECK(rayhit.normal[1] == Approx(sqrt(1) / 2));
      CHECK(rayhit.normal[2] == Approx(0.f));
    }
  }
}
*/
