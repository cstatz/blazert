//
// Created by raph on 05.05.20.
//

#include "../doctest.h"
#include "../test_helpers.h"
#include <blazert/datatypes.h>

using namespace blazert;
using namespace doctest;

TEST_CASE_TEMPLATE("IntersectRayAABB", T, float, double)
{
    SUBCASE("basic_case")
    {
    // highest numbers
    T pos_inf = std::numeric_limits<T>::max();
    T neg_inf = -1 * std::numeric_limits<T>::max();

    // return variables    
    T tmin;
    T tmax;    
    
    // correct values
    T tmin_cor = 1.5000000000000002;
    T tmax_cor = 2.519803902718557;
        
    // parameters
    T min_t = 0.0;
    T max_t = 3.0;
    Vec3r<T> bmin{ -1.0, 1.0, 1.0 };
    Vec3r<T> bmax{ 1.0, 2.0, 2.0 };
    Vec3r<T> ray_org{ 0.0, -0.4708710135363803, 1.794174202707276 };
    Vec3r<T> ray_inv_dir{ pos_inf, 1.019803902718557, -5.099019513592786 };
    Vec3ui ray_dir_sign{ 0, 0, 1 };
    
    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));
    REQUIRE(tmin == Approx(tmin_cor));
    REQUIRE(tmax == Approx(tmax_cor));
    }    
    
    SUBCASE("BB_point")
    {
    // highest numbers
    T pos_inf = std::numeric_limits<T>::max();
    T neg_inf = -1 * std::numeric_limits<T>::max();

    // return variables    
    T tmin;
    T tmax;    
    
    // correct values
    T tmin_cor = 1.7320508075688772;
    T tmax_cor = 1.7320508075688772;
        
    // parameters
    T min_t = 0.0;
    T max_t = 3.0;
    Vec3r<T> bmin{ -1.0, 1.0, 1.0 };
    Vec3r<T> bmax{ -1.0, 1.0, 1.0 };
    Vec3r<T> ray_org{ 0.0, 0.0, 0.0 };
    Vec3r<T> ray_inv_dir{ -1.0, 1.0, 1.0 };
    Vec3ui ray_dir_sign{ 1, 0, 0 };
    
    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));
    REQUIRE(tmin == Approx(tmin_cor));
    REQUIRE(tmax == Approx(tmax_cor));
    }    

    SUBCASE("BB_point_inf")
    {
    // highest numbers
    T pos_inf = std::numeric_limits<T>::max();
    T neg_inf = -1 * std::numeric_limits<T>::max();

    // return variables    
    T tmin;
    T tmax;    
    
    // correct values
    T tmin_cor = pos_inf;
    T tmax_cor = pos_inf;
        
    // parameters
    T min_t = 0.0;
    T max_t = pos_inf;
    Vec3r<T> bmin{ -1.0, pos_inf, 1.0 };
    Vec3r<T> bmax{ -1.0, pos_inf, 1.0 };
    Vec3r<T> ray_org{ 0.0, 0.0, 0.0 };
    Vec3r<T> ray_inv_dir{ -1.0, 0.0, 1.0 };
    Vec3ui ray_dir_sign{ 1, 0, 0 };
    
    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));
    REQUIRE(tmin == Approx(tmin_cor));
    REQUIRE(tmax == Approx(tmax_cor));
    }
    

    SUBCASE("BB_line")
    {
    // highest numbers
    T pos_inf = std::numeric_limits<T>::max();
    T neg_inf = -1 * std::numeric_limits<T>::max();

    // return variables    
    T tmin;
    T tmax;    
    
    // correct values
    T tmin_cor = 2.449489742783178;
    T tmax_cor = 2.449489742783178;
        
    // parameters
    T min_t = 0.0;
    T max_t = 3.0;
    Vec3r<T> bmin{ -1.0, 1.0, 1.0 };
    Vec3r<T> bmax{ -1.0, 5.0, 1.0 };
    Vec3r<T> ray_org{ 0.0, 0.0, 0.0 };
    Vec3r<T> ray_inv_dir{ -1.0, 0.5, 1.0 };
    Vec3ui ray_dir_sign{ 1, 0, 0 };
    
    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));
    REQUIRE(tmin == Approx(tmin_cor));
    REQUIRE(tmax == Approx(tmax_cor));
    }
    
    SUBCASE("BB_line_inf")
    {
    // highest numbers
    T pos_inf = std::numeric_limits<T>::max();
    T neg_inf = -1 * std::numeric_limits<T>::max();

    // return variables    
    T tmin;
    T tmax;    
    
    // correct values
    T tmin_cor = 2.449489742783178;
    T tmax_cor = 2.449489742783178;
        
    // parameters
    T min_t = 0.0;
    T max_t = 3.0;
    Vec3r<T> bmin{ -1.0, 1.0, 1.0 };
    Vec3r<T> bmax{ -1.0, pos_inf, 1.0 };
    Vec3r<T> ray_org{ 0.0, 0.0, 0.0 };
    Vec3r<T> ray_inv_dir{ -1.0, 0.5, 1.0 };
    Vec3ui ray_dir_sign{ 1, 0, 0 };
    
    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));
    REQUIRE(tmin == Approx(tmin_cor));
    REQUIRE(tmax == Approx(tmax_cor));
    }
    
    SUBCASE("BB_plane")
    {
    // highest numbers
    T pos_inf = std::numeric_limits<T>::max();
    T neg_inf = -1 * std::numeric_limits<T>::max();

    // return variables    
    T tmin;
    T tmax;    
    
    // correct values
    T tmin_cor = 3.7416573867739413;
    T tmax_cor = 3.7416573867739413;
        
    // parameters
    T min_t = 0.0;
    T max_t = 3.0;
    Vec3r<T> bmin{ -1.0, 1.0, 1.0 };
    Vec3r<T> bmax{ -1.0, 5.0, 5.0 };
    Vec3r<T> ray_org{ 0.0, 0.0, 0.0 };
    Vec3r<T> ray_inv_dir{ -1.0, 0.5, 0.3333333333333333 };
    Vec3ui ray_dir_sign{ 1, 0, 0 };
    
    REQUIRE(IntersectRayAABB(tmin, tmax, min_t, max_t, bmin, bmax, ray_org, ray_inv_dir, ray_dir_sign));
    REQUIRE(tmin == Approx(tmin_cor));
    REQUIRE(tmax == Approx(tmax_cor));
    }    
}