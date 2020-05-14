//
// Created by ogarten on 13/05/2020.
//

#include "../catch.hpp"
#include <blazert/embree/primitives/EmbreeSphere.h>
#include <blazert/embree/scene.h>

using namespace blazert;

TEMPLATE_TEST_CASE("EmbreeScene", "intersections]", float, double) {
  SECTION("Sphere") {
    Vec3r<TestType> center{0.f, 0.f, 0.f};
    float radius = 1.f;

    auto centers = std::make_unique<Vec3rList<TestType>>();
    auto radii = std::make_unique<std::vector<TestType>>();

    centers->emplace_back(center);
    radii->emplace_back(radius);

    SECTION("intersections") {
      SECTION("Ray origin outside sphere") {
        SECTION("hit in distance 1") {
          EmbreeScene<TestType> scene;
          unsigned int prim_id = scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<TestType> org1{2.f, 0.f, 0.f};
          Vec3r<TestType> dir1{-1.f, 0.f, 0.f};

          const Ray<TestType> ray1{org1, dir1};
          RayHit<TestType> rayhit1;
          const bool hit = intersect1(scene, ray1, rayhit1);
          // should be in distance of 1
          REQUIRE(hit);
          REQUIRE(rayhit1.prim_id == prim_id);
          REQUIRE(rayhit1.hit_distance == Approx(1.f));
          REQUIRE(rayhit1.normal[0] == Approx(1));
          REQUIRE(rayhit1.normal[1] == Approx(0));
          REQUIRE(rayhit1.normal[2] == Approx(0));
        }
        SECTION("hit in distance 2") {
          EmbreeScene<TestType> scene;
          unsigned int prim_id = scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<TestType> org3{0.f, 0.f, 3.f};
          Vec3r<TestType> dir3{0.f, 0.f, -1.f};
          const Ray<TestType> ray3{org3, dir3};
          RayHit<TestType> rayhit3;
          const bool hit = intersect1(scene, ray3, rayhit3);
          // should be in distance of 2
          REQUIRE(hit);
          REQUIRE(rayhit3.prim_id == prim_id);
          REQUIRE(rayhit3.hit_distance == Approx(2.f));
          REQUIRE(rayhit3.normal[0] == Approx(0));
          REQUIRE(rayhit3.normal[1] == Approx(0));
          REQUIRE(rayhit3.normal[2] == Approx(1));
        }
        SECTION("no hit")
        {
          EmbreeScene<TestType> scene;
          scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<TestType> org2{2.f, 0.f, 2.5f};
          Vec3r<TestType> dir2{-1.f, 0.f, 0.f};
          const Ray<TestType> ray2{org2, dir2};
          RayHit<TestType> rayhit2;
          const bool hit = intersect1(scene, ray2, rayhit2);
          // should not hit, therefore tfar is the same as before
          REQUIRE(!hit);
          REQUIRE(rayhit2.hit_distance == Approx(std::numeric_limits<TestType>::max()));
        }
      }
      SECTION("Ray origin inside sphere") {

        SECTION("origin = sphere center") {
          EmbreeScene<TestType> scene;
          unsigned int prim_id = scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<TestType> org1{0.f, 0.f, 0.f};
          Vec3r<TestType> dir1{-1.f, 0.f, 0.f};
          const Ray<TestType> ray1{org1, dir1};
          RayHit<TestType> rayhit1;
          const bool hit = intersect1(scene, ray1, rayhit1);
          // should be in distance of 1
          REQUIRE(hit);
          REQUIRE(rayhit1.prim_id == prim_id);
          REQUIRE(rayhit1.hit_distance == Approx(1.f));
          REQUIRE(rayhit1.normal[0] == Approx(-1));
          REQUIRE(rayhit1.normal[1] == Approx(0));
          REQUIRE(rayhit1.normal[2] == Approx(0));
        }
        SECTION("origin != sphere center") {
          EmbreeScene<TestType> scene;
          unsigned int prim_id = scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<TestType> org2{0.f, 0.f, 0.5f};
          Vec3r<TestType> dir2{0.f, 0.f, -1.f};
          const Ray<TestType> ray2{org2, dir2};
          RayHit<TestType> rayhit2;
          const bool hit = intersect1(scene, ray2, rayhit2);
          // should  hit in distance of 1.5
          REQUIRE(hit);
          REQUIRE(rayhit2.prim_id == prim_id);
          REQUIRE(rayhit2.hit_distance == Approx(1.5f));
          REQUIRE(rayhit2.normal[0] == Approx(0));
          REQUIRE(rayhit2.normal[1] == Approx(0));
          REQUIRE(rayhit2.normal[2] == Approx(-1.f));

        }
      }
      SECTION("Ray origin on sphere") {
        SECTION("shooting inside") {
          EmbreeScene<TestType> scene;
          unsigned int prim_id = scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<TestType> org1{1.f, 0.f, 0.f};
          Vec3r<TestType> dir1{-1.f, 0.f, 0.f};
          const Ray<TestType> ray1{org1, dir1};
          RayHit<TestType> rayhit1;
          const bool hit = intersect1(scene, ray1, rayhit1);
          // should hit in distance of 2
          REQUIRE(hit);
          REQUIRE(rayhit1.prim_id == prim_id);
          REQUIRE(rayhit1.hit_distance == Approx(2.f));
          REQUIRE(rayhit1.normal[0] == Approx(-1));
          REQUIRE(rayhit1.normal[1] == Approx(0));
          REQUIRE(rayhit1.normal[2] == Approx(0));
        }
        SECTION("shooting outside") {
          EmbreeScene<TestType> scene;
          scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<TestType> org2{1.f, 0.f, 0.f};
          Vec3r<TestType> dir2{1.f, 0.f, 0.f};
          const Ray<TestType> ray2{org2, dir2};
          RayHit<TestType> rayhit2;
          const bool hit = intersect1(scene, ray2, rayhit2);
          REQUIRE(!hit);
          REQUIRE(rayhit2.hit_distance == Approx(std::numeric_limits<TestType>::max()));
        }
      }
      SECTION("ray not passing through sphere center") {
        EmbreeScene<TestType> scene;
        unsigned int prim_id = scene.add_spheres(*centers, *radii);
        scene.commit();

        Vec3r<TestType> org1{2.f, 0.5f, 0.f};
        Vec3r<TestType> dir1{-1.f, 0.f, 0.f};
        const Ray<TestType> ray1{org1, dir1};
        RayHit<TestType> rayhit1;
        const bool hit = intersect1(scene, ray1, rayhit1);

        REQUIRE(hit);
        REQUIRE(rayhit1.prim_id == prim_id);
        REQUIRE(rayhit1.hit_distance == Approx(1.133974596));
        REQUIRE(rayhit1.normal[0] == Approx(sqrt(3) / 2));
        REQUIRE(rayhit1.normal[1] == Approx(sqrt(1) / 2));
        REQUIRE(rayhit1.normal[2] == Approx(0.f));
      }
    }
  }
  SECTION("Plane") {
    auto centers = std::make_unique<Vec3rList<TestType>>();
    auto dxs = std::make_unique<std::vector<TestType>>();
    auto dys = std::make_unique<std::vector<TestType>>();
    auto rotations = std::make_unique<Mat3rList<TestType>>();

    centers->emplace_back(Vec3r<TestType>{0., 0., 0.});
    dxs->emplace_back(2.);
    dys->emplace_back(2.);
    rotations->emplace_back(blaze::IdentityMatrix<TestType>(3UL));

    EmbreeScene<TestType> scene{};
    unsigned int prim_id = scene.add_planes(*centers, *dxs, *dys, *rotations);
    scene.commit();

    SECTION("intersections") {
      Vec3r<TestType> org{0.f, 0.f, 5.f};
      Vec3r<TestType> dir{0.f, 0.f, -1.f};

      const Ray<TestType> ray{org, dir};
      RayHit<TestType> rayhit;

      const bool hit = intersect1(scene, ray, rayhit);

      REQUIRE(hit);
      REQUIRE(rayhit.prim_id == prim_id);
      REQUIRE(rayhit.hit_distance == Approx(5));
      REQUIRE(rayhit.normal[0] == Approx(0));
      REQUIRE(rayhit.normal[1] == Approx(0));
      REQUIRE(rayhit.normal[2] == Approx(1));
    }
  }
}
