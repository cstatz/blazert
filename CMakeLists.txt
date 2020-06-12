CMAKE_MINIMUM_REQUIRED(VERSION 3.7.0)
PROJECT(blazert)

SET(ENABLE_OMP true CACHE BOOL "Enable OpenMP in examples.")
SET(BUILD_TEST false CACHE BOOL "Build tests.")
SET(BUILD_BENCHMARK false CACHE BOOL "Build benchmark.")
SET(BUILD_EXAMPLES false CACHE BOOL "Build examples.")
SET(EMBREE_BACKEND false CACHE BOOL "Use embree as tracing backend.")
SET(TEST_COVERAGE false CACHE BOOL "Test code coverage (using gcov, needs compilation with gcc).")
SET(BLAZE_INCLUDE_OVERRIDE "" CACHE STRING "Where to find the blaze includes. Must set on windows for blaze <= 3.7")

message("========================================")
message("= Configuring blazeRT:                 =")
message(" * Build test:      ${BUILD_TEST}")
message(" * Build benchmark: ${BUILD_BENCHMARK}")
message(" * Build examples:  ${BUILD_EXAMPLES}")
message(" * Embree backend:  ${EMBREE_BACKEND}")
message(" * Test coverage:   ${TEST_COVERAGE}")
IF (NOT ("${BLAZE_INCLUDE_OVERRIDE}" STREQUAL ""))
    message(" * Blaze include dir: ${BLAZE_INCLUDE_OVERRIDE}")
ENDIF ()
message("========================================")

IF(TEST_COVERAGE)
SET(COVERAGE_FLAGS "--coverage")
ELSE()
SET(COVERAGE_FLAGS "")
ENDIF()

# Compiler Flags
set(CMAKE_CXX_STANDARD 17)
IF (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall -fPIC -funroll-loops -march=native ${COVERAGE_FLAGS}")
ELSEIF (MSVC)
    # this currently works, but is not really the way to go..
    set(CMAKE_CXX_FLAGS "/std:c++17 /Wall /arch:AVX")
ELSE ()
    set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall -fPIC -funroll-loops -march=native -fno-finite-math-only -fno-signed-zeros -freciprocal-math -ffp-contract=fast")
ENDIF (CMAKE_COMPILER_IS_GNUCXX)

IF (FORCE_COLOR_OUTPUT)
    IF (CMAKE_COMPILER_IS_GNUCXX)
        add_compile_options(-fdiagnostics-color=always)
    ELSE ()
        add_compile_options(-fcolor-diagnostics)
    ENDIF ()
ENDIF (FORCE_COLOR_OUTPUT)

INCLUDE(CheckCXXCompilerFlag)
INCLUDE(GNUInstallDirs)


IF (NOT ("${BLAZE_INCLUDE_OVERRIDE}" STREQUAL ""))
    INCLUDE_DIRECTORIES(${BLAZE_INCLUDE_OVERRIDE})
ELSE ()
    FIND_PACKAGE(blaze 3.7)
    INCLUDE_DIRECTORIES(${BLAZE_INCLUDE_DIRS})
ENDIF ()

IF (EMBREE_BACKEND)
    find_package(embree 3.9)
ENDIF ()

FIND_PACKAGE(Threads)
FIND_PACKAGE(OpenMP)

IF (ENABLE_OMP)
    IF (OpenMP_CXX_FOUND)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        link_libraries(OpenMP::OpenMP_CXX)
        include_directories(${OpenMP_CXX_INCLUDE_DIRS})
    ENDIF ()
ENDIF ()

IF (BUILD_EXAMPLES)
    ADD_SUBDIRECTORY(examples)
ENDIF ()

SET(BLAZERT_VERSION_NUMBER 0.0.1)
SET(BLAZERT_VERSION_STRING ${BLAZERT_VERSION_NUMBER})
SET(BLAZERT_DEFINITIONS "")
SET(BLAZERT_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
SET(BLAZERT_ROOT_DIR ${CMAKE_INSTALL_PREFIX})

INCLUDE(CMakePackageConfigHelpers)

CONFIGURE_PACKAGE_CONFIG_FILE(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/blazert.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/blazertConfig.cmake
        PATH_VARS BLAZERT_INCLUDE_DIR BLAZERT_ROOT_DIR
        INSTALL_DESTINATION ${CMAKE_INSTALL_DATADIR}/blazert/cmake
        NO_CHECK_REQUIRED_COMPONENTS_MACRO)

WRITE_BASIC_PACKAGE_VERSION_FILE(blazertConfigVersion.cmake VERSION ${BLAZERT_VERSION_NUMBER} COMPATIBILITY SameMajorVersion)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/blazertConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/blazertConfigVersion.cmake DESTINATION ${CMAKE_INSTALL_DATADIR}/blazert/cmake)
INSTALL(DIRECTORY blazert DESTINATION include)

IF (BUILD_TEST)
    ENABLE_TESTING()
    FILE(GLOB_RECURSE TEST_SRC ${CMAKE_SOURCE_DIR}/test/*.cpp)
    IF (EMBREE_BACKEND)
        FILE(GLOB_RECURSE EMBREE_PRIMITIVES_SRC ${CMAKE_SOURCE_DIR}/test/test_embree_primitives/*.cpp)
    ELSE()
        LIST(REMOVE_ITEM TEST_SRC ${CMAKE_SOURCE_DIR}/test/test_embree_primitives/test_EmbreeCylinder.cpp)
        LIST(REMOVE_ITEM TEST_SRC ${CMAKE_SOURCE_DIR}/test/test_embree_primitives/test_EmbreePlane.cpp)
        LIST(REMOVE_ITEM TEST_SRC ${CMAKE_SOURCE_DIR}/test/test_embree_primitives/test_EmbreeScene.cpp)
        LIST(REMOVE_ITEM TEST_SRC ${CMAKE_SOURCE_DIR}/test/test_embree_primitives/test_EmbreeSphere.cpp)
    ENDIF ()
    ADD_EXECUTABLE(test_blazert ${TEST_SRC} ${EMBREE_PRIMITIVES_SRC})
    TARGET_INCLUDE_DIRECTORIES(test_blazert PUBLIC ${CMAKE_SOURCE_DIR} ${EMBREE_INCLUDE_DIRS} ${BLAZE_INCLUDE_DIRS} "/usr/local/include")
    TARGET_LINK_LIBRARIES(test_blazert ${EMBREE_LIBRARY})
    SET_TARGET_PROPERTIES(test_blazert PROPERTIES LINKER_LANGUAGE CXX)
    ADD_TEST(main test_blazert)

    find_package(ImageMagick COMPONENTS compare QUIET)
    if (NOT ImageMagick_compare_FOUND)
        message(WARNING "Cannot find ImageMagick's compare tool. Images will not be compared to the reference.")
    else ()
        add_test(
                NAME "render_trimesh"
                COMMAND
                ${CMAKE_COMMAND}
                -DCOMPARE_EXECUTABLE=${ImageMagick_compare_EXECUTABLE}
                -DRENDER_EXECUTABLE=${CMAKE_BINARY_DIR}/examples/renderer/renderer
                -DREFERENCE=${CMAKE_SOURCE_DIR}/examples/baseline/renderer.ppm
                -DOUTPUT=${CMAKE_SOURCE_DIR}/examples/render.ppm
                -DDIFFERENCE=${CMAKE_CURRENT_BINARY_DIR}/difference_renderer.png
                -P ${PROJECT_SOURCE_DIR}/cmake/compare_image_test.cmake
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/examples)
        add_test(
                NAME "render_other_primitives"
                COMMAND
                ${CMAKE_COMMAND}
                -DCOMPARE_EXECUTABLE=${ImageMagick_compare_EXECUTABLE}
                -DRENDER_EXECUTABLE=${CMAKE_BINARY_DIR}/examples/scene_primitives/scene_primitives
                -DREFERENCE=${CMAKE_SOURCE_DIR}/examples/baseline/scene_primitives.png
                -DOUTPUT=${CMAKE_SOURCE_DIR}/examples/render.png
                -DDIFFERENCE=${CMAKE_CURRENT_BINARY_DIR}/difference_scene_primitives.png
                -P ${PROJECT_SOURCE_DIR}/cmake/compare_image_test.cmake
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/examples)
    endif ()

ENDIF ()

IF (BUILD_BENCHMARK)
    FIND_PACKAGE(benchmark)
    file(GLOB_RECURSE BENCHMARK_SRC ${CMAKE_SOURCE_DIR}/benchmarks/*)
    add_executable(benchmark_blazeRT ${BENCHMARK_SRC})
    TARGET_INCLUDE_DIRECTORIES(benchmark_blazeRT PUBLIC ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/third_party/bvh/include ${CMAKE_SOURCE_DIR}/third_party/nanort ${BLAZE_INCLUDE_DIRS} ${EMBREE_INCLUDE_DIRS})
    target_link_libraries(benchmark_blazeRT PUBLIC benchmark::benchmark ${BLAZE_LIBRARY} ${LAPACK_LIBRARIES} ${EMBREE_LIBRARY})
ENDIF ()
