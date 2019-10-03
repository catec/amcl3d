/*!
 * @file Node.h
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

#include "Parameters.h"
#include "ParticleFilter.h"  //! Include Grid.hpp

#include <rosinrange_msg/RangePose.h>
#include <tf/transform_broadcaster.h>

namespace amcl3d
{
class Node
{
public:
  explicit Node();
  virtual ~Node();

  void spin();

private:
  void publishMapPointCloud(const ros::TimerEvent&);
  void publishGridSlice(const ros::TimerEvent&);
  void publishGridTf(const ros::TimerEvent&);
  void publishParticles();

  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void odomCallback(const geometry_msgs::TransformStampedConstPtr& msg);
  void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void rangeCallback(const rosinrange_msg::RangePoseConstPtr& msg);

  //! Check motion and time thresholds for AMCL update
  bool checkUpdateThresholds();

  //! Set the initial pose of the particle filter
  void setInitialPose(const tf::Transform& init_pose, const float x_dev, const float y_dev, const float z_dev,
                      const float a_dev);

  //! Return yaw from a given TF
  double getYawFromTf(const tf::Transform& tf);

  //! To show range sensors in Rviz
  void rvizMarkerPublish(const uint32_t anchor_id, const float r, const geometry_msgs::Point& uav,
                         const geometry_msgs::Point& anchor);

  Parameters parameters_;
  Grid3d grid3d_;
  ParticleFilter pf_;

  ros::NodeHandle nh_;

  sensor_msgs::PointCloud2 map_point_cloud_msg_;
  ros::Publisher map_point_cloud_pub_;
  ros::Timer map_point_cloud_pub_timer_;

  nav_msgs::OccupancyGrid grid_slice_msg_;
  ros::Publisher grid_slice_pub_;
  ros::Timer grid_slice_pub_timer_;

  tf::StampedTransform grid_to_world_tf_;
  ros::Timer grid_to_world_tf_timer_;

  bool is_odom_{ false };
  bool amcl_out_{ false };
  double roll_{ 0 }, pitch_{ 0 };

  std::vector<Range> range_data;
  Particle mean_p_, lastmean_p_;

  ros::Subscriber point_sub_, odom_sub_, range_sub_, initialpose_sub_;
  ros::Publisher particles_pose_pub_, range_markers_pub_, odom_base_pub_;

  tf::Transform lastbase_2_world_tf_, initodom_2_world_tf_, lastodom_2_world_tf_, amcl_out_lastbase_2_odom_tf_,
      lastupdatebase_2_odom_tf_, base_2_odom_tf_, odom_increment_tf;

  ros::Time nextupdate_time_;

  ros::Publisher cloud_filter_pub_;
};

}  // namespace amcl3d
