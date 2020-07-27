#define DOCTEST_CONFIG_INCLUDE_TYPE_TRAITS

#include "test_helpers.h"
#include <third_party/doctest/doctest/doctest.h>

#include <blazert/datatypes.h>

using namespace doctest;

TEST_CASE_TEMPLATE("Math", T, float, double) {
  CHECK(static_cast<T>(0.) == Approx(static_cast<T>(0.)));
  CHECK(static_cast<T>(1.) == Approx(static_cast<T>(1.)));

  CHECK_FALSE(static_cast<T>(0.) == static_cast<T>(1.));
  CHECK_FALSE(static_cast<T>(1.) == static_cast<T>(0.));

  CHECK_FALSE(static_cast<T>(0.) == std::numeric_limits<T>::lowest());
  CHECK_FALSE(std::numeric_limits<T>::lowest() == static_cast<T>(0.));

  CHECK_FALSE(static_cast<T>(0.) == std::numeric_limits<T>::max());
  CHECK_FALSE(static_cast<T>(0.) == std::numeric_limits<T>::min());
  CHECK_FALSE(std::numeric_limits<T>::max() == static_cast<T>(0.));
  CHECK_FALSE(std::numeric_limits<T>::min() == static_cast<T>(0.));

  CHECK(std::numeric_limits<T>::max() == Approx(std::numeric_limits<T>::max()));
  CHECK(std::numeric_limits<T>::min() == Approx(std::numeric_limits<T>::min()));

  CHECK_FALSE(std::numeric_limits<T>().max() + std::numeric_limits<T>().lowest()
              == Approx(std::numeric_limits<T>().max()));
}
