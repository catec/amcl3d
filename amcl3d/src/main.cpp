/*!
 * @file main.cpp
 * @copyright Copyright (c) 2019, FADA-CATEC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
