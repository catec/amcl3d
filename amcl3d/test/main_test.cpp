/*!
 * @file main_test.cpp
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
