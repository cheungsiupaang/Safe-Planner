cmake_minimum_required(VERSION 2.8.3)
project(safeplan)

# set the default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wno-deprecated-declarations")

execute_process(COMMAND lsb_release -cs
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
)

SET(COMPILE_ELLIPSOID_PLANNER_NODE TRUE)
if(${LSB_RELEASE_ID_SHORT} MATCHES "bionic")
  SET(COMPILE_ELLIPSOID_PLANNER_NODE FALSE)
  message("-- Systemm version: ${LSB_RELEASE_ID_SHORT}, disable the compile of [ellipsoid_planner_node]")
endif()
#message("-- COMPILE_ELLIPSOID_PLANNER_NODE: ${COMPILE_ELLIPSOID_PLANNER_NODE}")
find_package(PCL REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
find_package(jps3d REQUIRED)
find_package(planning_ros_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${DECOMP_UTIL_INCLUDE_DIRS} ${JPS3D_INCLUDE_DIRS})

catkin_simple()

add_executable(path_decomp_jps src/path_decomp_jps.cpp)
target_link_libraries(path_decomp_jps ${JPS3D_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} ${catkin_LIBRARIES})

cs_install()

cs_export()
