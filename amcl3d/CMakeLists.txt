project(amcl3d)
cmake_minimum_required(VERSION 3.5.1)

add_compile_options(-std=c++11)

##################################
## Configure CATKIN dependecies ##
##################################

find_package(catkin REQUIRED
  COMPONENTS
    visualization_msgs
    nav_msgs
    octomap_ros
    pcl_ros
    rosinrange_msg
)

###################################
## CATKIN specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    visualization_msgs
    nav_msgs
    octomap_ros
    pcl_ros
    rosinrange_msg
)

######################
## Configure OpenMP ##
######################

find_package(OpenMP  REQUIRED)

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

###########
## Build ##
###########

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}           src/Grid3d.cpp src/Grid3d.h src/ParticleFilter.cpp src/ParticleFilter.h)
add_dependencies(${PROJECT_NAME}      ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node         src/main.cpp src/Node.cpp src/Node.h src/Parameters.cpp src/Parameters.h)
add_dependencies(${PROJECT_NAME}_node       ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES} ${PROJECT_NAME})

file(GLOB_RECURSE DEMO_SRCS "demo/*.c*")
file(GLOB_RECURSE DEMO_HDRS "demo/*.h*")

add_executable(${PROJECT_NAME}_demo         ${DEMO_SRCS} ${DEMO_HDRS})
add_dependencies(${PROJECT_NAME}_demo       ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_demo  ${catkin_LIBRARIES})
