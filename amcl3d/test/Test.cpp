/*!
 * @file Test.cpp
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

#include "tf/transform_listener.h"

namespace amcl3d
{
Test::Test()
{
}

Test::~Test()
{
}

void Test::spin()
{
  ROS_DEBUG("[%s] Test::spin()", ros::this_node::getName().data());

  //! Initialize transforms and variables
  vicon_tf_.setIdentity();
  vicon_init_tf_.setIdentity();
  vicon_relative_tf_.setIdentity();

  //! Initialize subscribers and publishers
  vicon_sub_ = nh_.subscribe("vicon_client/ROSIN_F550/pose", 1, &Test::baseCallback, this);
  vicon_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("vicon_odometry", 1);

  pointcloud_sub_ = nh_.subscribe("/os1_cloud_node/points", 1, &Test::cloudCallback, this);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("lidar_pointcloud", 1);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  nh_.shutdown();
}

void Test::cloudCallback(const sensor_msgs::PointCloud2Ptr& msg)
{
  //! To correct publishing the pointcloud
  msg->header.stamp = ros::Time(0);
  msg->header.frame_id = "lidar_points";

  pointcloud_pub_.publish(msg);
}

void Test::baseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  //! To send the odometry from /odom

  vicon_tf_.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
  vicon_tf_.setRotation(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                                       msg->pose.orientation.w));

  if (!got_vicon_init_)
  {
    vicon_init_tf_.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));

    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                     +msg->pose.orientation.w);

    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    vicon_init_tf_.setRotation(tf::createQuaternionFromRPY(0, 0, yaw));

    got_vicon_init_ = true;
  }

  vicon_relative_tf_ = vicon_init_tf_.inverse() * vicon_tf_;

  tf::Vector3 position = vicon_relative_tf_.getOrigin();
  tf::Quaternion orientation = vicon_relative_tf_.getRotation();

  geometry_msgs::TransformStamped vicon_relative;
  vicon_relative.header.stamp.sec = ros::Time::now().sec;
  vicon_relative.header.stamp.nsec = ros::Time::now().nsec;
  vicon_relative.header.frame_id = "vicon_odometry";
  vicon_relative.transform.translation.x = position.getX();
  vicon_relative.transform.translation.y = position.getY();
  vicon_relative.transform.translation.z = position.getZ();
  vicon_relative.transform.rotation.x = orientation.getX();
  vicon_relative.transform.rotation.y = orientation.getY();
  vicon_relative.transform.rotation.z = orientation.getZ();
  vicon_relative.transform.rotation.w = orientation.getW();

  vicon_pub_.publish(vicon_relative);

  br_.sendTransform(tf::StampedTransform(vicon_tf_, ros::Time::now(), "world", "vicon_real"));
}
}  // namespace amcl3d
