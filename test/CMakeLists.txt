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

# standard compiler warnings
include(../cmake/compiler_warnings.cmake)
set_project_warnings(test_blazert PRIVATE)