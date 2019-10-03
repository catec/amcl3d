project(rosinrange_msg)
cmake_minimum_required(VERSION 2.8.3)

##################################
## Configure CATKIN dependecies ##
##################################

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    message_generation
)

#######################
## Generate messages ##
#######################

add_message_files(DIRECTORY msg)

##########################################
## Generate added messages and services ##
##########################################

generate_messages(
  DEPENDENCIES
    geometry_msgs
)

###################################
## CATKIN specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
)
