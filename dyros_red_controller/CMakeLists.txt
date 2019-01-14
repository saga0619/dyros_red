cmake_minimum_required(VERSION 2.8.3)
project(dyros_red_controller)

add_compile_options(-std=c++11 -Ofast)

find_package(catkin REQUIRED COMPONENTS
    dyros_red_msgs
    geometry_msgs
    roscpp
    roslib
    sensor_msgs
    mujoco_ros_msgs
    std_msgs
    realtime_tools
    eigen_conversions    
    tf
    )
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
    DEPENDS   dyros_red_msgs geometry_msgs roscpp sensor_msgs std_msgs realtime_tools eigen_conversions mujoco_ros_msgs tf
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
    )
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )

file(GLOB BRIDGE_SRC
    "src/simulation_interface.cpp"
    "src/control_base.cpp"
    "src/mujoco_interface.cpp"
    )
file(GLOB CONTROLLER_SRC
    "src/dyros_red_model.cpp"
    "src/task_controller.cpp"
    "src/wholebody_controller.cpp"
    "src/quadraticprogram.cpp"
    )

#file(GLOB CVXGEN_SRC
#    "include/cvxgen/*.h"
#    "include/cvxgen/solver.c"
#    "include/cvxgen/matrix_support.c"
#    "include/cvxgen/ldl.c"
#    "include/cvxgen/util.c"
#    )


add_library(dyros_red_controller_lib
    ${BRIDGE_SRC}
    ${CONTROLLER_SRC}
    #  ${CVXGEN_SRC}
    #  ${UPPER_SRC}
    )

add_executable(dyros_red_controller src/dyros_red_controller.cpp)
target_link_libraries(dyros_red_controller_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}

    -L/usr/local/lib
    -lrbdl
    -lrbdl_urdfreader

    -lqpOASES
    -lrt
    )
target_link_libraries(dyros_red_controller
    dyros_red_controller_lib
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${CVXGEN_SRC}
    )

add_dependencies(dyros_red_controller_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(dyros_red_controller ${catkin_EXPORTED_TARGETS})

#SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE Debug)
