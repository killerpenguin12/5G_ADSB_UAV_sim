cmake_minimum_required(VERSION 2.8.3)
project(common_cpp)

# default to release build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)

include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(common_cpp INTERFACE)
target_include_directories(common_cpp INTERFACE include)

add_executable(unit_tests
  src/unit_tests.cpp
)
target_link_libraries(unit_tests common_cpp ${Eigen3_LIBRARIES})
