#cmake_minimum_required(VERSION 3.7)
#project("core")
#set(CMAKE_CXX_STANDARD 17)
set(BUILD_TARGET "bistatic_pattern_po")

include_directories(${CMAKE_SOURCE_DIR} "${CMAKE_SOURCE_DIR}/examples/common" "/usr/local/include" "/usr/include" "../" "../../" "../../data_interface")

add_executable(${BUILD_TARGET} main.cc ../../../../common/cnpy.cc ../../../../common/numpy_wrapper.h ../../constants.h)


find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(ZLIB REQUIRED)
target_include_directories(${BUILD_TARGET} PUBLIC ${ZLIB_INCLUDE_DIRS})

target_link_libraries(${BUILD_TARGET} PUBLIC nlohmann_json::nlohmann_json ${ZLIB_LIBRARIES})

source_group("Source Files" FILES ${SOURCES})
set_project_warnings(${BUILD_TARGET} PRIVATE OFF)
