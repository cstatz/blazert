#include <third_party/doctest/doctest/doctest.h>
#include "test_helpers.h"

#include <blazert/datatypes.h>

using namespace doctest;

TEST_CASE_TEMPLATE("Math", T, float, double) {
  REQUIRE(static_cast<T>(0.) == Approx(static_cast<T>(0.)));
  REQUIRE(static_cast<T>(1.) == Approx(static_cast<T>(1.)));

  REQUIRE_FALSE(static_cast<T>(0.) == static_cast<T>(1.));
  REQUIRE_FALSE(static_cast<T>(1.) == static_cast<T>(0.));

  REQUIRE_FALSE(0. == std::numeric_limits<T>::lowest());
  REQUIRE_FALSE(std::numeric_limits<T>::lowest() == 0.);
  
  REQUIRE_FALSE(0.f == std::numeric_limits<T>::max());
  REQUIRE_FALSE(0.f == std::numeric_limits<T>::min());
  REQUIRE_FALSE(std::numeric_limits<T>::max() == 0.f);
  REQUIRE_FALSE(std::numeric_limits<T>::min() == 0.f);

  REQUIRE(std::numeric_limits<T>::max() == Approx(std::numeric_limits<T>::max()));
  REQUIRE(std::numeric_limits<T>::min() == Approx(std::numeric_limits<T>::min()));
  
  REQUIRE_FALSE(std::numeric_limits<T>().max() + std::numeric_limits<T>().lowest() == Approx(std::numeric_limits<T>().max()));
}
