cmake_minimum_required(VERSION 2.8.3)
project(air_dynamics)

find_package(catkin_simple REQUIRED)

catkin_simple()

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
if (NOT TARGET wind-dynamics)
  include_directories(lib/wind-dynamics/include)
  add_subdirectory(lib/wind-dynamics)
endif()

cs_add_executable(air_dynamics_node
  src/air_dynamics_node.cpp
  src/air_dynamics.cpp
)
target_link_libraries(air_dynamics_node wind-dynamics utils ${catkin_LIBRARIES})

cs_install()
cs_export()
