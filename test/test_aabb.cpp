//
// Created by raph on 05.05.20.
//

#include "catch.hpp"
#include "test_helpers.h"
#include <blazert/datatypes.h>

using namespace blazert;

TEST_CASE("IntersectRayAABB", "[float, double]")
{
  {
    // return variables
    float tmin;
    float tmax;

    // parameters
    float min_t = 0.1;
    float max_t = 0.1;
    Vec3r<float> bmin{ 0, 0, 0 };
    Vec3r<float> bmax{ 0, 0, 1 };
    Vec3r<float> ray_org{ 0, 0, -1 };
    Vec3r<float> ray_inv_dir{ 0, 0, 1 };
    Vec3ui ray_dir_sign{ 3, 3, 3 };

    REQUIRE_FALSE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));

  }
  {
    // return variables
    double tmin;
    double tmax;

    // parameters
    double min_t = -5;
    double max_t = 5;
    double pos_inf = std::numeric_limits<double>().max();
    double neg_inf = -1 * std::numeric_limits<double>().max();
    Vec3r<double> bmin{ 1, -1, -1 };
    Vec3r<double> bmax{ 1, 1, 1 };
    Vec3r<double> ray_org{ 0, 0, 0 };
    Vec3r<double> ray_inv_dir{1, pos_inf, pos_inf};
    Vec3ui ray_dir_sign{ 0, 0, 0 };

    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));

  }
}

