option(${PROJECT_NAME}_ENABLE_OMP "Enable OpenMP in examples." OFF)
option(${PROJECT_NAME}_BUILD_TEST "Build tests." OFF)
option(${PROJECT_NAME}_BUILD_BENCHMARK "Build benchmark." OFF)
option(${PROJECT_NAME}_BUILD_EXAMPLES "Build examples." OFF)
option(${PROJECT_NAME}_EMBREE_BACKEND "Use embree as tracing backend." OFF)
option(${PROJECT_NAME}_TEST_COVERAGE "Test code coverage (using gcov, needs compilation with gcc)." OFF)
set(BLAZE_INCLUDE_OVERRIDE "" CACHE STRING "Where to find the blaze includes. Must be set on windows for blaze <= 3.7")

message("========================================")
message("= Configuring blazeRT:                 =")
message(" * Build type:       ${CMAKE_BUILD_TYPE}")
message(" * Build test:       ${${PROJECT_NAME}_BUILD_TEST}")
message(" * Build benchmark:  ${${PROJECT_NAME}_BUILD_BENCHMARK}")
message(" * Build examples:   ${${PROJECT_NAME}_BUILD_EXAMPLES}")
message(" * \t|> with OpenMP:  ${${PROJECT_NAME}_ENABLE_OMP}")
message(" * Embree backend:   ${${PROJECT_NAME}_EMBREE_BACKEND}")
message(" * Test coverage:    ${${PROJECT_NAME}_TEST_COVERAGE}")
if(NOT ("${BLAZE_INCLUDE_OVERRIDE}" STREQUAL ""))
    message(" * Blaze include dir: ${BLAZE_INCLUDE_OVERRIDE}")
endif()
message("========================================")