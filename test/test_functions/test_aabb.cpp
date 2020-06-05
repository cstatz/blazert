//
// Created by raph on 05.05.20.
//

#include <third_party/doctest/doctest/doctest.h>
#include "../test_helpers.h"
#include <blazert/datatypes.h>
//#include <blazert/bvh/aabb.h>
//#include <blazert/bvh/accel.h>

using namespace blazert;
using namespace doctest;


TEST_CASE_TEMPLATE("Intersect_node", T, float, double) {
  // highest numbers
  T pos_inf = std::numeric_limits<T>::max();
  T neg_inf = -1 * std::numeric_limits<T>::max();

  SUBCASE("basic_case") {
    // return variables
    T tmin;
    T tmax;

    // correct values
    const T tmin_cor = 1.5000000000000002;
    const T tmax_cor = 2.519803902718557;

    const Vec3r<T> bmin{-1.0, 1.0, 1.0};
    const Vec3r<T> bmax{1.0, 2.0, 2.0};

    BVHNode<T> node;
    node.min = bmin;
    node.max = bmax;

    // parameters
    const T min_t = 0.0;
    const T max_t = 3.0;
    const Vec3r<T> ray_org{0.0, -0.4708710135363803, 1.794174202707276};
    // ray_dir from former const Vec3r<T> ray_inv_dir{pos_inf, 1.019803902718557, -5.099019513592786};
    const Vec3r<T> ray_dir{0, 0.9805806756909201, -0.19611613513818398692};
    const Ray<T> ray{ray_org, ray_dir, min_t, max_t};


    Vec3ui ray_dir_sign{0, 0, 1};

    const bool hit = intersect_node(tmin, tmax, node, ray);
    CHECK(hit);
    CHECK(tmin == Approx(tmin_cor));
    CHECK(tmax == Approx(tmax_cor));
  }

  
  SUBCASE("point-like bounding box") {
    SUBCASE("finite point") {
      // return variables
      T tmin;
      T tmax;

      // correct values
      T tmin_cor = 1;
      1.7320508075688772;
      T tmax_cor = 1;
      1.7320508075688772;

      // parameters
      T min_t = 0.0;
      T max_t = 3.0;
      Vec3r<T> bmin{-1.0, 1.0, 1.0};
      Vec3r<T> bmax{-1.0, 1.0, 1.0};
      Vec3r<T> ray_org{0.0, 0.0, 0.0};
      Vec3r<T> ray_inv_dir{-1.0, 1.0, 1.0};
      Vec3ui ray_dir_sign{1, 0, 0};

      const bool hit = IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign);
      CHECK(hit);
      CHECK(tmin == Approx(tmin_cor));
      CHECK(tmax == Approx(tmax_cor));
    }

    SUBCASE("y-infinite point") {
      // return variables
      T tmin;
      T tmax;

      // parameters
      T min_t = 0.0;
      T max_t = pos_inf;
      Vec3r<T> bmin{-1.0, pos_inf, 1.0};
      Vec3r<T> bmax{-1.0, pos_inf, 1.0};
      Vec3r<T> ray_org{0.0, 0.0, 0.0};
      Vec3r<T> ray_inv_dir{-1.0, 0.0, 1.0};
      Vec3ui ray_dir_sign{1, 0, 0};

      const bool hit = IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign);
      CHECK_FALSE(hit);
    }
  }
  SUBCASE("line-like bounding box") {
    SUBCASE("finite line") {
      // return variables
      T tmin;
      T tmax;

      // correct values
      T tmin_cor = 2.449489742783178;
      T tmax_cor = 2.449489742783178;

      // parameters
      T min_t = 0.0;
      T max_t = 3.0;
      Vec3r<T> bmin{-1.0, 1.0, 1.0};
      Vec3r<T> bmax{-1.0, 5.0, 1.0};
      Vec3r<T> ray_org{0.0, 0.0, 0.0};
      Vec3r<T> ray_inv_dir{-1.0, 0.5, 1.0};
      Vec3ui ray_dir_sign{1, 0, 0};

      const bool hit = IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign);
      CHECK(hit);
      CHECK(tmin == Approx(tmin_cor));
      CHECK(tmax == Approx(tmax_cor));
    }

    SUBCASE("infinite line") {
      // return variables
      T tmin;
      T tmax;

      // correct values
      T tmin_cor = 2.449489742783178;
      T tmax_cor = 2.449489742783178;

      // parameters
      T min_t = 0.0;
      T max_t = 3.0;
      Vec3r<T> bmin{-1.0, 1.0, 1.0};
      Vec3r<T> bmax{-1.0, pos_inf, 1.0};
      Vec3r<T> ray_org{0.0, 0.0, 0.0};
      Vec3r<T> ray_inv_dir{-1.0, 0.5, 1.0};
      Vec3ui ray_dir_sign{1, 0, 0};

      const bool hit = IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign);
      CHECK(hit);
      CHECK(tmin == Approx(tmin_cor));
      CHECK(tmax == Approx(tmax_cor));
    }
  }
  SUBCASE("plane-like bounding box") {
    SUBCASE("finite plane") {
      // return variables
      T tmin;
      T tmax;

      // correct values
      T tmin_cor = 3.7416573867739413;
      T tmax_cor = 3.7416573867739413;

      // parameters
      T min_t = 0.0;
      T max_t = 3.0;
      Vec3r<T> bmin{-1.0, 1.0, 1.0};
      Vec3r<T> bmax{-1.0, 5.0, 5.0};
      Vec3r<T> ray_org{0.0, 0.0, 0.0};
      Vec3r<T> ray_inv_dir{-1.0, 0.5, 0.3333333333333333};
      Vec3ui ray_dir_sign{1, 0, 0};

      const bool hit = IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign);
      CHECK(hit);
      CHECK(tmin == Approx(tmin_cor));
      CHECK(tmax == Approx(tmax_cor));
    }
  }
  
}