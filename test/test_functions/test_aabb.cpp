//
// Created by raph on 05.05.20.
//

#include "../doctest.h"
#include "../test_helpers.h"
#include <blazert/datatypes.h>

using namespace blazert;
using namespace doctest;

TEST_CASE("IntersectRayAABB")
{
    SUBCASE("float_example")
    {
    // return variables    
    float tmin;
    float tmax;    
    
    // correct values
    float tmin_cor = 1.5;
    float tmax_cor = 2.519804;
    
    // precision
    float epsilon = std::numeric_limits<float>().epsilon();
    
    // parameters
    float min_t = 0.0;
    float max_t = 3.0;
    float pos_inf = std::numeric_limits<float>().max();
    float neg_inf = -1 * std::numeric_limits<float>().max();
    Vec3r<float> bmin{ -1.0, 1.0, 1.0 };
    Vec3r<float> bmax{ 1.0, 2.0, 2.0 };
    Vec3r<float> ray_org{ 0.0, -0.470871, 1.7941742 };
    Vec3r<float> ray_inv_dir{ pos_inf, 1.0198039, -5.0990195 };
    Vec3ui ray_dir_sign{ 0, 0, 1 };
    
    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));
    REQUIRE(tmin == Approx(tmin_cor).epsilon(epsilon));
    REQUIRE(tmax == Approx(tmax_cor).epsilon(epsilon));
    }
    

    SUBCASE("double_example")
    {
    // return variables    
    double tmin;
    double tmax;    
    
    // correct values
    double tmin_cor = 1.5000000000000002;
    double tmax_cor = 2.519803902718557;
    
    // precision
    double epsilon = std::numeric_limits<double>().epsilon();
    
    // parameters
    double min_t = 0.0;
    double max_t = 3.0;
    double pos_inf = std::numeric_limits<double>().max();
    double neg_inf = -1 * std::numeric_limits<double>().max();
    Vec3r<double> bmin{ -1.0, 1.0, 1.0 };
    Vec3r<double> bmax{ 1.0, 2.0, 2.0 };
    Vec3r<double> ray_org{ 0.0, -0.4708710135363803, 1.794174202707276 };
    Vec3r<double> ray_inv_dir{ pos_inf, 1.019803902718557, -5.099019513592786 };
    Vec3ui ray_dir_sign{ 0, 0, 1 };
    
    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));
    REQUIRE(tmin == Approx(tmin_cor).epsilon(epsilon));
    REQUIRE(tmax == Approx(tmax_cor).epsilon(epsilon));
    }
}