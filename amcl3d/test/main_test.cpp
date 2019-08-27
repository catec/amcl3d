#include "../test/Test.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");

  ROS_INFO("[%s] Test initialization.", ros::this_node::getName().data());

  if (!ros::master::check())
  {
    ROS_ERROR("[%s] roscore is not running.", ros::this_node::getName().data());
    return EXIT_FAILURE;
  }

  amcl3d::Test node;
  node.spin();

  ROS_INFO("[%s] Test finished.", ros::this_node::getName().data());

  return EXIT_SUCCESS;
}
