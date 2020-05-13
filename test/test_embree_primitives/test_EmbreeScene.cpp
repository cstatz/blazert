//
// Created by ogarten on 13/05/2020.
//
#include "../catch.hpp"
#include <blazert/embree/primitives/EmbreeSphere.h>
#include <blazert/embree/scene.h>

using namespace blazert;

TEMPLATE_TEST_CASE("EmbreeScene_Sphere", "intersections]", float, double) {

  SECTION("INTERSECTIONS") {
    SECTION("Ray origin outside sphere") {
      Vec3r<TestType> center1{0.f, 0.f, 0.f};
      float radius = 1.f;

      auto centers = std::make_unique<Vec3rList<TestType>>();
      auto radii = std::make_unique<std::vector<TestType>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      EmbreeScene<TestType> scene{};

      scene.add_spheres(*centers, *radii);
      scene.commit();

      Vec3r<TestType> org1{2.f, 0.f, 0.f};
      Vec3r<TestType> dir1{-1.f, 0.f, 0.f};

      const Ray<TestType> ray1{org1, dir1};
      RayHit<TestType> rayhit1;
      intersect1(scene, ray1, rayhit1);
      // should be in distance of 1
      REQUIRE(rayhit1.hit_distance == Approx(1.f));

      Vec3r<TestType> org2{2.f, 0.f, 2.5f};
      Vec3r<TestType> dir2{-1.f, 0.f, 0.f};
      const Ray<TestType> ray2{org2, dir2};
      RayHit<TestType> rayhit2;
      intersect1(scene, ray2, rayhit2);
      // should not hit, therefore tfar is the same as before
      REQUIRE(rayhit2.hit_distance == Approx(std::numeric_limits<TestType>::max()));

      Vec3r<TestType> org3{0.f, 0.f, 3.f};
      Vec3r<TestType> dir3{0.f, 0.f, -1.f};
      const Ray<TestType> ray3{org3, dir3};
      RayHit<TestType> rayhit3;
      intersect1(scene, ray3, rayhit3);
      // should be in distance of 2
      REQUIRE(rayhit3.hit_distance == Approx(2.f));
    }

    SECTION("Ray origin inside sphere") {
      Vec3r<TestType> center1{0.f, 0.f, 0.f};
      float radius = 1.f;
      auto centers = std::make_unique<Vec3rList<TestType>>();
      auto radii = std::make_unique<std::vector<TestType>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      EmbreeScene<TestType> scene{};
      scene.add_spheres(*centers, *radii);
      scene.commit();

      Vec3r<TestType> org1{0.f, 0.f, 0.f};
      Vec3r<TestType> dir1{-1.f, 0.f, 0.f};
      const Ray<TestType> ray1{org1, dir1};
      RayHit<TestType> rayhit1;
      intersect1(scene, ray1, rayhit1);
      // should be in distance of 1
      REQUIRE(rayhit1.hit_distance == Approx(1.f));

      Vec3r<TestType> org2{0.f, 0.f, 0.5f};
      Vec3r<TestType> dir2{0.f, 0.f, -1.f};
      const Ray<TestType> ray2{org2, dir2};
      RayHit<TestType> rayhit2;
      intersect1(scene, ray2, rayhit2);
      // should  hit in distance of 1.5
      REQUIRE(rayhit2.hit_distance == Approx(1.5f));
    }
    SECTION("Ray origin on sphere")
      ///
    {
      Vec3r<TestType> center1{1.f, 0.f, 0.f};
      float radius = 1.f;
      auto centers = std::make_unique<Vec3rList<TestType>>();
      auto radii = std::make_unique<std::vector<TestType>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      EmbreeScene<TestType> scene{};
      scene.add_spheres(*centers, *radii);
      scene.commit();

      Vec3r<TestType> org1{0.f, 0.f, 0.f};
      Vec3r<TestType> dir1{1.f, 0.f, 0.f};
      const Ray<TestType> ray1{org1, dir1};
      RayHit<TestType> rayhit1;
      intersect1(scene, ray1, rayhit1);
      // should hit in distance of 2
      REQUIRE(rayhit1.hit_distance == Approx(2.f));

      Vec3r<TestType> org2{0.f, 0.f, 0.f};
      Vec3r<TestType> dir2{-1.f, 0.f, 0.f};
      const Ray<TestType> ray2{org2, dir2};
      RayHit<TestType> rayhit2;
      intersect1(scene, ray2, rayhit2);
      REQUIRE(rayhit2.hit_distance == Approx(std::numeric_limits<TestType>::max()));
    }
    SECTION("ray passing through sphere center") {
      Vec3r<TestType> center1{0.f, 0.f, 0.f};
      float radius = 2.f;

      auto centers = std::make_unique<Vec3rList<TestType>>();
      auto radii = std::make_unique<std::vector<TestType>>();
      centers->emplace_back(center1);
      radii->emplace_back(radius);

      EmbreeScene<TestType> scene{};
      scene.add_spheres(*centers, *radii);
      scene.commit();

      Vec3r<TestType> org1{3.f, 1.f, 0.f};
      Vec3r<TestType> dir1{-1.f, 0.f, 0.f};
      const Ray<TestType> ray1{org1, dir1};
      RayHit<TestType> rayhit1;
      intersect1(scene, ray1, rayhit1);

      REQUIRE(rayhit1.hit_distance == Approx(1.26794919f));

      REQUIRE(rayhit1.normal[0] == Approx(sqrt(3) / 2));
      REQUIRE(rayhit1.normal[1] == Approx(sqrt(1) / 2));
      REQUIRE(rayhit1.normal[2] == Approx(0.f));
    }
  }
}