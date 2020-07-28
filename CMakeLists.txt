CMAKE_MINIMUM_REQUIRED(VERSION 3.7.0)
PROJECT(blazert CXX)

# include build option flags
include(cmake/build_options.cmake)

# turn on test coverage
IF(TEST_COVERAGE)
    SET(COVERAGE_FLAGS "--coverage")
ELSE()
    SET(COVERAGE_FLAGS "")
ENDIF()

# add library target
add_library(blazeRT INTERFACE)
file(GLOB_RECURSE BLAZERT_FILES blazert/*.h)
target_sources(blazeRT INTERFACE ${BLAZERT_FILES})
target_include_directories(blazeRT INTERFACE blazert/)
include(cmake/compiler_warnings.cmake)
set_project_warnings(blazeRT INTERFACE)

# Compiler Flags
set(CMAKE_CXX_STANDARD 17)
IF (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -fPIC -funroll-loops -march=native ${COVERAGE_FLAGS}")
ELSEIF (MSVC)
    # this currently works, but is not really the way to go..
    set(CMAKE_CXX_FLAGS "/std:c++17 /Wall /arch:AVX")
ELSE ()
    set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -fPIC -funroll-loops -march=native -fno-finite-math-only -fno-signed-zeros -freciprocal-math -ffp-contract=fast")
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

IF (ENABLE_OMP)
    FIND_PACKAGE(OpenMP)
    IF (OpenMP_CXX_FOUND)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        link_libraries(OpenMP::OpenMP_CXX)
        include_directories(${OpenMP_CXX_INCLUDE_DIRS})
    ENDIF ()
ENDIF ()

IF (BUILD_TEST)
    ENABLE_TESTING()
    add_subdirectory(test)

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

IF (BUILD_EXAMPLES)
    ADD_SUBDIRECTORY(examples)
ENDIF ()

IF (BUILD_BENCHMARK)
    add_subdirectory(benchmarks)
ENDIF ()

SET(BLAZERT_VERSION_NUMBER 20.2.1)
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




