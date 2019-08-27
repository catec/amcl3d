#include "Node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl3d_node");

  ROS_INFO("[%s] Node initialization.", ros::this_node::getName().data());

  if (!ros::master::check())
  {
    ROS_ERROR("[%s] Roscore is not running.", ros::this_node::getName().data());
    return EXIT_FAILURE;
  }

  amcl3d::Node node;
  node.spin();

  ROS_INFO("[%s] Node finished.", ros::this_node::getName().data());

  return EXIT_SUCCESS;
}
