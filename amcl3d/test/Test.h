/*!
 * @file Test.h
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

#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace amcl3d
{
class Test
{
public:
  explicit Test();
  virtual ~Test();

  void spin();

private:
  void cloudCallback(const sensor_msgs::PointCloud2Ptr& msg);
  void baseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  ros::NodeHandle nh_;

  ros::Subscriber vicon_sub_;
  ros::Subscriber pointcloud_sub_;

  ros::Publisher vicon_pub_;
  ros::Publisher pointcloud_pub_;

  bool got_vicon_init_{ false };

  tf::Transform vicon_tf_;
  tf::Transform vicon_init_tf_;
  tf::Transform vicon_relative_tf_;

  tf::TransformBroadcaster br_;
};
}  // namespace amcl3d
