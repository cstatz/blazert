//
// Created by ogarten on 13/05/2020.
//
/*
#include <third_party/doctest/doctest/doctest.h>
#include <blazert/embree/primitives/EmbreeSphere.h>
#include <blazert/embree/scene.h>

using namespace blazert;
using namespace doctest;


TEST_CASE_TEMPLATE("EmbreeScene", T, float, double) {
  SUBCASE("Sphere") {
    Vec3r<T> center{0.f, 0.f, 0.f};
    float radius = 1.f;

    auto centers = std::make_unique<Vec3rList<T>>();
    auto radii = std::make_unique<std::vector<T>>();

    centers->emplace_back(center);
    radii->emplace_back(radius);

    SUBCASE("intersections") {
      SUBCASE("Ray origin outside sphere") {
        SUBCASE("hit in distance 1") {
          EmbreeScene<T> scene;
          unsigned int prim_id = scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<T> org1{2.f, 0.f, 0.f};
          Vec3r<T> dir1{-1.f, 0.f, 0.f};

          const Ray<T> ray1{org1, dir1};
          RayHit<T> rayhit1;
          const bool hit = intersect1(scene, ray1, rayhit1);
          // should be in distance of 1
          CHECK(hit);
          CHECK(rayhit1.prim_id == prim_id);
          CHECK(rayhit1.hit_distance == Approx(1.f));
          CHECK(rayhit1.normal[0] == Approx(1));
          CHECK(rayhit1.normal[1] == Approx(0));
          CHECK(rayhit1.normal[2] == Approx(0));
        }
        SUBCASE("hit in distance 2") {
          EmbreeScene<T> scene;
          unsigned int prim_id = scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<T> org3{0.f, 0.f, 3.f};
          Vec3r<T> dir3{0.f, 0.f, -1.f};
          const Ray<T> ray3{org3, dir3};
          RayHit<T> rayhit3;
          const bool hit = intersect1(scene, ray3, rayhit3);
          // should be in distance of 2
          CHECK(hit);
          CHECK(rayhit3.prim_id == prim_id);
          CHECK(rayhit3.hit_distance == Approx(2.f));
          CHECK(rayhit3.normal[0] == Approx(0));
          CHECK(rayhit3.normal[1] == Approx(0));
          CHECK(rayhit3.normal[2] == Approx(1));
        }
        SUBCASE("no hit") {
          EmbreeScene<T> scene;
          scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<T> org2{2.f, 0.f, 2.5f};
          Vec3r<T> dir2{-1.f, 0.f, 0.f};
          const Ray<T> ray2{org2, dir2};
          RayHit<T> rayhit2;
          const bool hit = intersect1(scene, ray2, rayhit2);
          // should not hit, therefore tfar is the same as before
          CHECK(!hit);
          CHECK(rayhit2.hit_distance == Approx(std::numeric_limits<T>::max()));
        }
      }
      SUBCASE("Ray origin inside sphere") {

        SUBCASE("origin = sphere center") {
          EmbreeScene<T> scene;
          unsigned int prim_id = scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<T> org1{0.f, 0.f, 0.f};
          Vec3r<T> dir1{-1.f, 0.f, 0.f};
          const Ray<T> ray1{org1, dir1};
          RayHit<T> rayhit1;
          const bool hit = intersect1(scene, ray1, rayhit1);
          // should be in distance of 1
          CHECK(hit);
          CHECK(rayhit1.prim_id == prim_id);
          CHECK(rayhit1.hit_distance == Approx(1.f));
          CHECK(rayhit1.normal[0] == Approx(-1));
          CHECK(rayhit1.normal[1] == Approx(0));
          CHECK(rayhit1.normal[2] == Approx(0));
        }
        SUBCASE("origin != sphere center") {
          EmbreeScene<T> scene;
          unsigned int prim_id = scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<T> org2{0.f, 0.f, 0.5f};
          Vec3r<T> dir2{0.f, 0.f, -1.f};
          const Ray<T> ray2{org2, dir2};
          RayHit<T> rayhit2;
          const bool hit = intersect1(scene, ray2, rayhit2);
          // should  hit in distance of 1.5
          CHECK(hit);
          CHECK(rayhit2.prim_id == prim_id);
          CHECK(rayhit2.hit_distance == Approx(1.5f));
          CHECK(rayhit2.normal[0] == Approx(0));
          CHECK(rayhit2.normal[1] == Approx(0));
          CHECK(rayhit2.normal[2] == Approx(-1.f));
        }
      }
      SUBCASE("Ray origin on sphere") {
        SUBCASE("shooting inside") {
          EmbreeScene<T> scene;
          unsigned int prim_id = scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<T> org1{1.f, 0.f, 0.f};
          Vec3r<T> dir1{-1.f, 0.f, 0.f};
          const Ray<T> ray1{org1, dir1};
          RayHit<T> rayhit1;
          const bool hit = intersect1(scene, ray1, rayhit1);
          // should hit in distance of 2
          CHECK(hit);
          CHECK(rayhit1.prim_id == prim_id);
          CHECK(rayhit1.hit_distance == Approx(2.f));
          CHECK(rayhit1.normal[0] == Approx(-1));
          CHECK(rayhit1.normal[1] == Approx(0));
          CHECK(rayhit1.normal[2] == Approx(0));
        }
        SUBCASE("shooting outside") {
          EmbreeScene<T> scene;
          scene.add_spheres(*centers, *radii);
          scene.commit();

          Vec3r<T> org2{1.f, 0.f, 0.f};
          Vec3r<T> dir2{1.f, 0.f, 0.f};
          const Ray<T> ray2{org2, dir2};
          RayHit<T> rayhit2;
          const bool hit = intersect1(scene, ray2, rayhit2);
          CHECK(!hit);
          CHECK(rayhit2.hit_distance == Approx(std::numeric_limits<T>::max()));
        }
      }
      SUBCASE("ray not passing through sphere center") {
        EmbreeScene<T> scene;
        unsigned int prim_id = scene.add_spheres(*centers, *radii);
        scene.commit();

        Vec3r<T> org1{2.f, 0.5f, 0.f};
        Vec3r<T> dir1{-1.f, 0.f, 0.f};
        const Ray<T> ray1{org1, dir1};
        RayHit<T> rayhit1;
        const bool hit = intersect1(scene, ray1, rayhit1);

        CHECK(hit);
        CHECK(rayhit1.prim_id == prim_id);
        CHECK(rayhit1.hit_distance == Approx(1.133974596));
        CHECK(rayhit1.normal[0] == Approx(sqrt(3) / 2));
        CHECK(rayhit1.normal[1] == Approx(sqrt(1) / 2));
        CHECK(rayhit1.normal[2] == Approx(0.f));
      }
    }
  }
  SUBCASE("Plane") {
    auto centers = std::make_unique<Vec3rList<T>>();
    auto dxs = std::make_unique<std::vector<T>>();
    auto dys = std::make_unique<std::vector<T>>();
    auto rotations = std::make_unique<Mat3rList<T>>();

    centers->emplace_back(Vec3r<T>{0., 0., 0.});
    dxs->emplace_back(2.);
    dys->emplace_back(2.);
    rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));

    EmbreeScene<T> scene{};
    unsigned int prim_id = scene.add_planes(*centers, *dxs, *dys, *rotations);
    scene.commit();

    SUBCASE("intersections") {
      Vec3r<T> org{0.f, 0.f, 5.f};
      Vec3r<T> dir{0.f, 0.f, -1.f};

      const Ray<T> ray{org, dir};
      RayHit<T> rayhit;

      const bool hit = intersect1(scene, ray, rayhit);

      CHECK(hit);
      CHECK(rayhit.prim_id == prim_id);
      CHECK(rayhit.hit_distance == Approx(5));
      CHECK(rayhit.normal[0] == Approx(0));
      CHECK(rayhit.normal[1] == Approx(0));
      CHECK(rayhit.normal[2] == Approx(1));
    }
  }
  SUBCASE("Cylinder") {
    auto centers = std::make_unique<Vec3rList<T>>();
    auto semi_axes_a = std::make_unique<std::vector<T>>();
    auto semi_axes_b = std::make_unique<std::vector<T>>();
    auto heights = std::make_unique<std::vector<T>>();
    auto rotations = std::make_unique<Mat3rList<T>>();

    centers->emplace_back(Vec3r<T>{0., 0., 0.});
    semi_axes_a->emplace_back(2.);
    semi_axes_b->emplace_back(2.);
    heights->emplace_back(2.);
    rotations->emplace_back(blaze::IdentityMatrix<T>(3UL));

    EmbreeScene<T> scene{};
    unsigned int prim_id = scene.add_cylinders(*centers, *semi_axes_a, *semi_axes_b, *heights, *rotations);
    scene.commit();

    SUBCASE("intersections") {
      Vec3r<T> org{0.f, 0.f, 5.f};
      Vec3r<T> dir{0.f, 0.f, -1.f};

      const Ray<T> ray{org, dir};
      RayHit<T> rayhit;

      const bool hit = intersect1(scene, ray, rayhit);

      CHECK(hit);
      CHECK(rayhit.prim_id == prim_id);
      CHECK(rayhit.hit_distance == Approx(3));
      CHECK(rayhit.normal[0] == Approx(0));
      CHECK(rayhit.normal[1] == Approx(0));
      CHECK(rayhit.normal[2] == Approx(1));
    }
  }
}
 */
