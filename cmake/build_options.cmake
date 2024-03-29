option(ENABLE_OMP "Enable OpenMP in examples." OFF)
option(BUILD_TEST "Build tests." OFF)
option(BUILD_BENCHMARK "Build benchmark." OFF)
option(BUILD_EXAMPLES "Build examples." OFF)
option(EMBREE_BACKEND "Use embree as tracing backend." OFF)
option(TEST_COVERAGE "Test code coverage (using gcov, needs compilation with gcc)." OFF)
set(BLAZE_INCLUDE_OVERRIDE "" CACHE STRING "Where to find the blaze includes. Must be set on windows for blaze <= 3.7")

message("========================================")
message("= Configuring blazeRT:                 =")
message(" * Build type:       ${CMAKE_BUILD_TYPE}")
message(" * Build test:       ${BUILD_TEST}")
message(" * Build benchmark:  ${BUILD_BENCHMARK}")
message(" * Build examples:   ${BUILD_EXAMPLES}")
message(" * \t|> with OpenMP:  ${ENABLE_OMP}")
message(" * Embree backend:   ${EMBREE_BACKEND}")
message(" * Test coverage:    ${TEST_COVERAGE}")
if(NOT ("${BLAZE_INCLUDE_OVERRIDE}" STREQUAL ""))
    message(" * Blaze include dir: ${BLAZE_INCLUDE_OVERRIDE}")
endif()
message("========================================")