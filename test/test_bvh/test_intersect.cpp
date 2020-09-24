//
// Created by raph on 05.05.20.
//

#define DOCTEST_CONFIG_INCLUDE_TYPE_TRAITS

#include "../test_helpers.h"
#include <blazert/bvh/intersect.h>
#include <blazert/bvh/node.h>
#include <blazert/datatypes.h>
#include <blazert/primitives/plane.h>
#include <third_party/doctest/doctest/doctest.h>

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("Intersect_node", T, float, double) {
  // highest numbers
  T pos_inf = std::numeric_limits<T>::max();

  SUBCASE("basic case") {
    // return variables
    T tmin = 0.0;
    T tmax = 3.0;

    // correct values
    const T tmin_cor = 1.5;
    const T tmax_cor = static_cast<T>(2.519803902718557);

    // parameters
    const T min_t = 0.0;
    const T max_t = 3.0;
    const Vec3r<T> bmin{-1.0, 1.0, 1.0};
    const Vec3r<T> bmax{1.0, 2.0, 2.0};
    const Vec3r<T> ray_org{0.0, static_cast<T>(-0.4708710135363803), static_cast<T>(1.794174202707276)};
    const Vec3r<T> ray_dir{0.0, static_cast<T>(0.9805806756909201), static_cast<T>(-0.196116135138184)};

    // initialize node
    BVHNode<T, PlaneCollection> node;// could be any collection
    node.min = bmin;
    node.max = bmax;

    // build ray
    const Ray<T> ray{ray_org, ray_dir, min_t, max_t};

    // calculate result
    const bool hit = intersect_node(tmin, tmax, node, ray);

    CHECK(hit);
    CHECK(tmin == Approx(tmin_cor));
    CHECK(tmax == Approx(tmax_cor));
  }

  SUBCASE("point-like bounding box") {
    SUBCASE("finite point") {
      // return variables
      T tmin = 0.0;
      T tmax = 3.0;

      // correct values
      const T tmin_cor = static_cast<T>(1.7320508075688772);
      const T tmax_cor = static_cast<T>(1.7320508075688772);

      // parameters
      const T min_t = 0.0;
      const T max_t = 3.0;
      const Vec3r<T> bmin{-1.0, 1.0, 1.0};
      const Vec3r<T> bmax{-1.0, 1.0, 1.0};
      const Vec3r<T> ray_org{0.0, 0.0, 0.0};
      const Vec3r<T> ray_dir{static_cast<T>(-0.5773502691896258), static_cast<T>(0.5773502691896258),
                             static_cast<T>(0.5773502691896258)};

      // initialize node
      BVHNode<T, PlaneCollection> node;// could be any collection
      node.min = bmin;
      node.max = bmax;

      // build ray
      const Ray<T> ray{ray_org, ray_dir, min_t, max_t};

      // calculate result
      const bool hit = intersect_node(tmin, tmax, node, ray);

      CHECK(hit);
      CHECK(tmin == Approx(tmin_cor));
      CHECK(tmax == Approx(tmax_cor));
    }

    SUBCASE("y-infinite point") {
      // return variables
      T tmin = 0.0;
      T tmax = pos_inf;

      // correct values
      const T tmin_cor = pos_inf;
      const T tmax_cor = pos_inf;

      // parameters
      const T min_t = 0.0;
      const T max_t = pos_inf;
      const Vec3r<T> bmin{-1.0, pos_inf, 1.0};
      const Vec3r<T> bmax{-1.0, pos_inf, 1.0};
      const Vec3r<T> ray_org{0.0, 0.0, 0.0};
      const Vec3r<T> ray_dir{static_cast<T>(-0.0009999990000015), static_cast<T>(0.9999990000015),
                             static_cast<T>(0.0009999990000015)};

      // initialize node
      BVHNode<T, PlaneCollection> node;// could be any collection
      node.min = bmin;
      node.max = bmax;

      // build ray
      const Ray<T> ray{ray_org, ray_dir, min_t, max_t};

      // calculate result
      const bool hit = intersect_node(tmin, tmax, node, ray);

      WARN(hit);
      WARN(tmin == Approx(tmin_cor));
      WARN(tmax == Approx(tmax_cor));
    }
  }

  SUBCASE("line-like bounding box") {
    SUBCASE("finite line") {
      // return variables
      T tmin = 0.0;
      T tmax = 3.0;

      // correct values
      const T tmin_cor = static_cast<T>(2.449489742783178);
      const T tmax_cor = static_cast<T>(2.449489742783178);

      // parameters
      const T min_t = 0.0;
      const T max_t = 3.0;
      const Vec3r<T> bmin{-1.0, 1.0, 1.0};
      const Vec3r<T> bmax{-1.0, 5.0, 1.0};
      const Vec3r<T> ray_org{0.0, 0.0, 0.0};
      const Vec3r<T> ray_dir{static_cast<T>(-0.4082482904638631), static_cast<T>(0.8164965809277261),
                             static_cast<T>(0.4082482904638631)};

      // initialize node
      BVHNode<T, PlaneCollection> node;// could be any collection
      node.min = bmin;
      node.max = bmax;

      // build ray
      const Ray<T> ray{ray_org, ray_dir, min_t, max_t};

      // calculate result
      const bool hit = intersect_node(tmin, tmax, node, ray);

      CHECK(hit);
      CHECK(tmin == Approx(tmin_cor));
      CHECK(tmax == Approx(tmax_cor));
    }

    SUBCASE("y-infinite line") {
      // return variables
      T tmin = 0.0;
      T tmax = 3.0;

      // correct values
      const T tmin_cor = static_cast<T>(2.449489742783178);
      const T tmax_cor = static_cast<T>(2.449489742783178);

      // parameters
      const T min_t = 0.0;
      const T max_t = 3.0;
      const Vec3r<T> bmin{-1.0, 1.0, 1.0};
      const Vec3r<T> bmax{-1.0, pos_inf, 1.0};
      const Vec3r<T> ray_org{0.0, 0.0, 0.0};
      const Vec3r<T> ray_dir{static_cast<T>(-0.4082482904638631), static_cast<T>(0.8164965809277261),
                             static_cast<T>(0.4082482904638631)};

      // initialize node
      BVHNode<T, PlaneCollection> node;// could be any collection
      node.min = bmin;
      node.max = bmax;

      // build ray
      const Ray<T> ray{ray_org, ray_dir, min_t, max_t};

      // calculate result
      const bool hit = intersect_node(tmin, tmax, node, ray);

      CHECK(hit);
      CHECK(tmin == Approx(tmin_cor));
      CHECK(tmax == Approx(tmax_cor));
    }
  }

  SUBCASE("plane-like bounding box") {
    SUBCASE("finite plane") {
      // return variables
      T tmin = 0.0;
      T tmax = 5.0;

      // correct values
      const T tmin_cor = static_cast<T>(3.7416573867739413);
      const T tmax_cor = static_cast<T>(3.7416573867739413);

      // parameters
      const T min_t = 0.0;
      const T max_t = 5.0;
      const Vec3r<T> bmin{-1.0, 1.0, 1.0};
      const Vec3r<T> bmax{-1.0, 5.0, 5.0};
      const Vec3r<T> ray_org{0.0, 0.0, 0.0};
      const Vec3r<T> ray_dir{static_cast<T>(-0.2672612419124244), static_cast<T>(0.5345224838248488),
                             static_cast<T>(0.8017837257372732)};

      // initialize node
      BVHNode<T, PlaneCollection> node;// could be any collection
      node.min = bmin;
      node.max = bmax;

      // build ray
      const Ray<T> ray{ray_org, ray_dir, min_t, max_t};

      // calculate result
      const bool hit = intersect_node(tmin, tmax, node, ray);

      CHECK(hit);
      CHECK(tmin == Approx(tmin_cor));
      CHECK(tmax == Approx(tmax_cor));
    }
    SUBCASE("y-infinite plane") {
      // return variables
      T tmin = 0.0;
      T tmax = pos_inf;

      // correct values
      const T tmin_cor = static_cast<T>(2.692582403567252);
      const T tmax_cor = static_cast<T>(2.692582403567252);

      // parameters
      const T min_t = 0.0;
      const T max_t = pos_inf;
      const Vec3r<T> bmin{-1.0, 1.0, 1.0};
      const Vec3r<T> bmax{-1.0, pos_inf, 2.0};
      const Vec3r<T> ray_org{0.0, 0.0, 0.0};
      const Vec3r<T> ray_dir{static_cast<T>(-0.3713906763541037), static_cast<T>(0.7427813527082074),
                             static_cast<T>(0.5570860145311556)};

      // initialize node
      BVHNode<T, PlaneCollection> node;// could be any collection
      node.min = bmin;
      node.max = bmax;

      // build ray
      const Ray<T> ray{ray_org, ray_dir, min_t, max_t};

      // calculate result
      const bool hit = intersect_node(tmin, tmax, node, ray);

      CHECK(hit);
      CHECK(tmin == Approx(tmin_cor));
      CHECK(tmax == Approx(tmax_cor));
    }
    SUBCASE("yz-infinite plane") {
      // return variables
      T tmin = 0.0;
      T tmax = pos_inf;

      // correct values
      const T tmin_cor = static_cast<T>(2.692582403567252);
      const T tmax_cor = static_cast<T>(2.692582403567252);

      // parameters
      const T min_t = 0.0;
      const T max_t = pos_inf;
      const Vec3r<T> bmin{-1.0, 1.0, 1.0};
      const Vec3r<T> bmax{-1.0, pos_inf, pos_inf};
      const Vec3r<T> ray_org{0.0, 0.0, 0.0};
      const Vec3r<T> ray_dir{static_cast<T>(-0.3713906763541037), static_cast<T>(0.7427813527082074),
                             static_cast<T>(0.5570860145311556)};

      // initialize node
      BVHNode<T, PlaneCollection> node;// could be any collection
      node.min = bmin;
      node.max = bmax;

      // build ray
      const Ray<T> ray{ray_org, ray_dir, min_t, max_t};

      // calculate result
      const bool hit = intersect_node(tmin, tmax, node, ray);

      CHECK(hit);
      CHECK(tmin == Approx(tmin_cor));
      CHECK(tmax == Approx(tmax_cor));
    }
  }
}