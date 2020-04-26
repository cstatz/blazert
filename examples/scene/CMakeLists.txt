set(BUILD_TARGET "scene")

include_directories(${CMAKE_SOURCE_DIR} "${CMAKE_SOURCE_DIR}/examples/common" "/usr/local/include")

file(GLOB SOURCES "*.cc" "*.h")
add_executable(${BUILD_TARGET} ${SOURCES})
IF(OpenMP_CXX_FOUND)
target_link_libraries(${BUILD_TARGET} PUBLIC OpenMP::OpenMP_CXX)
ENDIF()

source_group("Source Files" FILES ${SOURCES})
