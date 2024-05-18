set(BUILD_TARGET "emrtpo_to_numpy")

include_directories(${CMAKE_SOURCE_DIR} "${CMAKE_SOURCE_DIR}/examples/common" "/usr/local/include" "/usr/include" "../" "../data_interface" "../core")

add_executable(${BUILD_TARGET} converter.cc ../../../common/cnpy.cc)


find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(ZLIB REQUIRED)
target_include_directories(${BUILD_TARGET} PUBLIC ${ZLIB_INCLUDE_DIRS})

target_link_libraries(${BUILD_TARGET} PUBLIC nlohmann_json::nlohmann_json ${ZLIB_LIBRARIES})

source_group("Source Files" FILES ${SOURCES})
set_project_warnings(${BUILD_TARGET} PRIVATE OFF)
