file(GLOB_RECURSE TEST_SRC ${CMAKE_SOURCE_DIR}/test/*.cpp)

IF (EMBREE_BACKEND)
   file(GLOB_RECURSE EMBREE_PRIMITIVES_SRC ${CMAKE_SOURCE_DIR}/test/test_embree_primitives/*.cpp)
ELSE()
    list(REMOVE_ITEM TEST_SRC ${CMAKE_SOURCE_DIR}/test/test_embree_primitives/test_EmbreeCylinder.cpp)
    list(REMOVE_ITEM TEST_SRC ${CMAKE_SOURCE_DIR}/test/test_embree_primitives/test_EmbreePlane.cpp)
    list(REMOVE_ITEM TEST_SRC ${CMAKE_SOURCE_DIR}/test/test_embree_primitives/test_EmbreeScene.cpp)
    list(REMOVE_ITEM TEST_SRC ${CMAKE_SOURCE_DIR}/test/test_embree_primitives/test_EmbreeSphere.cpp)
ENDIF ()


add_executable(test_blazert ${TEST_SRC} ${EMBREE_PRIMITIVES_SRC})
target_include_directories(test_blazert PUBLIC ${CMAKE_SOURCE_DIR} ${EMBREE_INCLUDE_DIRS} ${BLAZE_INCLUDE_DIRS} "/usr/local/include")
target_link_libraries(test_blazert PUBLIC ${EMBREE_LIBRARY})
target_link_libraries(test_blazert PRIVATE blazeRT)

set_target_properties(test_blazert PROPERTIES LINKER_LANGUAGE CXX)
add_test(main test_blazert)

# standard compiler warnings
include(../cmake/compiler_warnings.cmake)
set_project_warnings(test_blazert PRIVATE ${BLAZERT_WARNINGS_AS_ERRORS})