CMAKE_MINIMUM_REQUIRED(VERSION 3.7.0)
PROJECT(blazert)

# Compiler Flags
set(CMAKE_CXX_STANDARD 17)
IF(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -fPIC -funroll-loops -march=native")
ELSEIF(MSVC)
    # this currently work but is not really the way to go..
    set(CMAKE_CXX_FLAGS "/std:c++17 /Wall /arch:AVX")
ELSE()
    set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -fPIC -funroll-loops -march=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

IF(FORCE_COLOR_OUTPUT)
    IF(CMAKE_COMPILER_IS_GNUCXX)
        add_compile_options(-fdiagnostics-color=always)
    ELSE()
        add_compile_options (-fcolor-diagnostics)
    ENDIF()
ENDIF(FORCE_COLOR_OUTPUT)

INCLUDE(CheckCXXCompilerFlag)
INCLUDE(GNUInstallDirs)

FIND_PACKAGE(blaze REQUIRED 3.7)
INCLUDE_DIRECTORIES(${BLAZE_INCLUDE_DIRS})

FIND_PACKAGE(embree 3.0)
FIND_PACKAGE(benchmark REQUIRED)
#target_link_libraries(MyTarget benchmark::benchmark Threads::Threads benchmark::benchmark_main)
FIND_PACKAGE(Threads)

FIND_PACKAGE(OpenMP)
IF(OpenMP_CXX_FOUND)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF()

ADD_SUBDIRECTORY(examples)

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

FILE(GLOB_RECURSE TEST_SRC ${CMAKE_SOURCE_DIR}/test/*.cpp)

ADD_EXECUTABLE(test_blazert ${TEST_SRC})
TARGET_INCLUDE_DIRECTORIES(test_blazert PUBLIC ${CMAKE_SOURCE_DIR} ${BLAZE_INCLUDE_DIRS} "/usr/local/include")
SET_TARGET_PROPERTIES(test_blazert PROPERTIES LINKER_LANGUAGE CXX)
ADD_TEST(main test_blazert)


file(GLOB_RECURSE BENCHMARK_SRC ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/* )

add_executable(benchmark_blazert ${BENCHMARK_SRC})
TARGET_INCLUDE_DIRECTORIES(benchmark_blazert PUBLIC ${CMAKE_SOURCE_DIR} ${BLAZE_INCLUDE_DIRS})
target_link_libraries(benchmark_blazert PUBLIC benchmark::benchmark ${BLAZE_LIBRARY} ${LAPACK_LIBRARIES})

