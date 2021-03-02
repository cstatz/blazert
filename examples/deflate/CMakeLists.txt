set(BUILD_TARGET "deflate")

include_directories(${CMAKE_SOURCE_DIR} "${CMAKE_SOURCE_DIR}/examples/common" "/usr/local/include")

file(GLOB SOURCES "*.cc" "${CMAKE_SOURCE_DIR}/examples/common/cnpy.cc")
add_executable(${BUILD_TARGET} ${SOURCES})
message(${SOURCES})

find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(ZLIB REQUIRED)
target_include_directories(${BUILD_TARGET} PUBLIC ${ZLIB_INCLUDE_DIRS})

target_link_libraries(${BUILD_TARGET} PUBLIC nlohmann_json::nlohmann_json ${ZLIB_LIBRARIES})

IF (EMBREE_FOUND)
    target_compile_definitions(${BUILD_TARGET} PRIVATE EMBREE_TRACING)
    target_link_libraries(${BUILD_TARGET} PUBLIC ${EMBREE_LIBRARY})
    target_include_directories(${BUILD_TARGET} PUBLIC ${EMBREE_INCLUDE_DIRS})
ENDIF ()

IF (OpenMP_CXX_FOUND)
    target_link_libraries(${BUILD_TARGET} PUBLIC OpenMP::OpenMP_CXX)
ENDIF ()

source_group("Source Files" FILES ${SOURCES})
