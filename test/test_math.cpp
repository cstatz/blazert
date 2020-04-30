#include "catch.hpp"
#include "test_helpers.h"

#include <blazert/datatypes.h>

TEST_CASE("Math", "[comparisons, blaze]")
{
	REQUIRE(0. == Approx( 0.));
	REQUIRE(0.f == Approx( 0.f));
	REQUIRE(1. == Approx( 1.));
	REQUIRE(1.f == Approx( 1.f));

	REQUIRE_FALSE(0. == Approx( 1.));
	REQUIRE_FALSE(0.f == Approx( 1.f));
	REQUIRE_FALSE(1. == Approx( 0.));
	REQUIRE_FALSE(1.f == Approx( 0.f));

	REQUIRE_FALSE(0. == Approx( std::numeric_limits<double>().lowest()));
	REQUIRE_FALSE(std::numeric_limits<double>().lowest() == Approx( 0.));
	REQUIRE_FALSE(0.f == Approx( std::numeric_limits<float>().lowest()));
	REQUIRE_FALSE(std::numeric_limits<float>().lowest() == Approx( 0.f));

	REQUIRE_FALSE(0.f == Approx( std::numeric_limits<float>().max()));
	REQUIRE_FALSE(0.f == Approx( std::numeric_limits<float>().min()));
	REQUIRE_FALSE(std::numeric_limits<float>().max() == Approx( 0.f));
	REQUIRE_FALSE(std::numeric_limits<float>().min() == Approx( 0.f));

	REQUIRE(std::numeric_limits<double>().max() == Approx( std::numeric_limits<double>().max()));
	REQUIRE(std::numeric_limits<double>().min() == Approx( std::numeric_limits<double>().min()));
	REQUIRE(std::numeric_limits<double>().max() == Approx( std::numeric_limits<double>().max()));
	REQUIRE(std::numeric_limits<double>().min() == Approx( std::numeric_limits<double>().min()));

	REQUIRE_FALSE(std::numeric_limits<float>().max() + std::numeric_limits<float>().lowest() == Approx( std::numeric_limits<float>().max()));
	REQUIRE_FALSE(std::numeric_limits<double>().max() + std::numeric_limits<double>().lowest() == Approx( std::numeric_limits<double>().max()));
}
