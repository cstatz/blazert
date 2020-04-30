//
// Created by Christoph Statz on 27.04.20.
//

#include "pmmintrin.h"
#include "xmmintrin.h"

#define CATCH_CONFIG_RUNNER

#include "catch.hpp"
#include "test_helpers.h"

int
main(int argc, char* const argv[])
{
  // global setup...

  _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

  int result = Catch::Session().run(argc, argv);

  return result;
}