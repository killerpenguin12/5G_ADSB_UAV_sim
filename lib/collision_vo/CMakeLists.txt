cmake_minimum_required(VERSION 3.7)
project(collision_vo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message("-- Build type: Release")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  # ${CERES_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC
        include/collision_vo/collision_vo.h
        src/collision_vo.cpp

        include/collision_vo/utils.h
        src/utils.cpp

        include/collision_vo/admissible_velocities.h
        src/admissible_velocities.cpp

        include/collision_vo/convex_hull.h
        src/convex_hull.cpp

        # include/collision_vo/min_solve.h
        # src/min_solve.cpp

        include/collision_vo/buffer.h
        src/buffer.cpp
        )


# add_library(${PROJECT_NAME} INTERFACE)
# target_include_directories(${PROJECT_NAME} INTERFACE include)

# option(BUILD_COLLISION_TESTS "Build Collision Tests" OFF)
# if(BUILD_COLLISION_TESTS)
#   message(STATUS, "Building Tests")
#   add_definitions(-DBUILD_COLLISION_TESTS)
# 	find_package(GTest REQUIRED)
# 	include_directories(${GTEST_INCLUDE_DIRS})
# 	add_executable(collision_tests tests/collision_tests.cpp)
# 	target_link_libraries(collision_tests ${GTEST_LIBRARIES} gtest gtest_main pthread ${PROJECT_NAME} ${CERES_LIBRARIES})
#
# 	add_executable(utils_tests tests/utils_tests.cpp)
# 	target_link_libraries(utils_tests ${GTEST_LIBRARIES} gtest gtest_main pthread ${PROJECT_NAME})
#
#   add_executable(convex_hull_tests tests/convex_hull_tests.cpp)
#   target_link_libraries(convex_hull_tests ${GTEST_LIBRARIES} gtest gtest_main pthread ${PROJECT_NAME})
#
#   add_executable(admissible_tests tests/admissible_tests.cpp)
#   target_link_libraries(admissible_tests ${GTEST_LIBRARIES} gtest gtest_main pthread ${PROJECT_NAME})
#
#   add_executable(min_solve_tests tests/min_solve_tests.cpp)
#   target_link_libraries(min_solve_tests ${GTEST_LIBRARIES} ${PROJECT_NAME} ${CERES_LIBRARIES} gtest gtest_main pthread)
#
#   add_executable(buffer_tests tests/buffer_test.cpp)
#   target_link_libraries(buffer_tests ${GTEST_LIBRARIES} ${PROJECT_NAME} ${CERES_LIBRARIES} gtest gtest_main pthread)
#
#   # if(OpenMP_CXX_FOUND)
#   #   message("Found OpenMP")
#   #   target_link_libraries(collision_tests PUBLIC OpenMP::OpenMP_CXX ${GTEST_LIBRARIES} gtest gtest_main pthread)
#   # endif()
# endif()
