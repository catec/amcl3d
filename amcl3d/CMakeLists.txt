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

###################
## Configure GSL ##
###################

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSL REQUIRED gsl)

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
   DEPENDS GSL
)

###########
## Build ##
###########

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${GSL_INCLUDE_DIRS})

file(GLOB_RECURSE UTILS_SRCS     "src/*.c*")
file(GLOB_RECURSE UTILS_HDRS     "src/*.h*")

add_executable(${PROJECT_NAME}_node          ${UTILS_SRCS} ${UTILS_HDRS})
add_dependencies(${PROJECT_NAME}_node        ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node   ${catkin_LIBRARIES} ${GSL_LIBRARIES})

file(GLOB_RECURSE TEST_SRCS      "test/*.c*")
file(GLOB_RECURSE TEST_HDRS      "test/*.h*")

add_executable(${PROJECT_NAME}_testnode         ${TEST_SRCS} ${TEST_HDRS})
add_dependencies(${PROJECT_NAME}_testnode       ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_testnode  ${catkin_LIBRARIES} ${GSL_LIBRARIES})