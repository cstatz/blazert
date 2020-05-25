//
// Created by raph on 05.05.20.
//

#include "../doctest.h"
#include "../test_helpers.h"
#include <blazert/datatypes.h>

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("IntersectRayAABB", T, double) {
  // highest numbers
  T pos_inf = std::numeric_limits<T>::max();
  T neg_inf = -1 * std::numeric_limits<T>::max();
  SUBCASE("basic_case") {
    // return variables
    T tmin;
    T tmax;

    // correct values
    T tmin_cor = 1.5000000000000002;
    T tmax_cor = 2.519803902718557;

    // parameters
    T min_t = 0.0;
    T max_t = 3.0;
    Vec3r<T> bmin{-1.0, 1.0, 1.0};
    Vec3r<T> bmax{1.0, 2.0, 2.0};
    Vec3r<T> ray_org{0.0, -0.4708710135363803, 1.794174202707276};
    Vec3r<T> ray_inv_dir{pos_inf, 1.019803902718557, -5.099019513592786};
    Vec3ui ray_dir_sign{0, 0, 1};

    const bool hit = IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign);
    REQUIRE(hit);
    REQUIRE(tmin == Approx(tmin_cor));
    REQUIRE(tmax == Approx(tmax_cor));
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
      REQUIRE(hit);
      REQUIRE(tmin == Approx(tmin_cor));
      REQUIRE(tmax == Approx(tmax_cor));
    }

    SUBCASE("y-infinite point") {
      // return variables
      T tmin;
      T tmax;

      // correct values
      T tmin_cor = pos_inf;
      T tmax_cor = pos_inf;

      // parameters
      T min_t = 0.0;
      T max_t = pos_inf;
      Vec3r<T> bmin{-1.0, pos_inf, 1.0};
      Vec3r<T> bmax{-1.0, pos_inf, 1.0};
      Vec3r<T> ray_org{0.0, 0.0, 0.0};
      Vec3r<T> ray_inv_dir{-1.0, 0.0, 1.0};
      Vec3ui ray_dir_sign{1, 0, 0};

      const bool hit = IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign);
      REQUIRE(hit);
      REQUIRE(tmin == Approx(tmin_cor));
      REQUIRE(tmax == Approx(tmax_cor));
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
      REQUIRE(hit);
      REQUIRE(tmin == Approx(tmin_cor));
      REQUIRE(tmax == Approx(tmax_cor));
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
      REQUIRE(hit);
      REQUIRE(tmin == Approx(tmin_cor));
      REQUIRE(tmax == Approx(tmax_cor));
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
      REQUIRE(hit);
      REQUIRE(tmin == Approx(tmin_cor));
      REQUIRE(tmax == Approx(tmax_cor));
    }
  }
}