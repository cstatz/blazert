//
// Created by Christoph Statz on 27.04.20.
//

#include "pmmintrin.h"
#include "xmmintrin.h"

#define DOCTEST_CONFIG_IMPLEMENT

#include <third_party/doctest/doctest/doctest.h>
#include "test_helpers.h"

int
main(int argc, char* const argv[])
{
  // global setup...
  _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

  doctest::Context context;
  context.applyCommandLine(argc, argv);
  int res = context.run(); // run

  if(context.shouldExit()) // important - query flags (and --exit) rely on the user doing this
    return res;          // propagate the result of the tests
}