/*!
 * @file Node.cpp
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

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>

namespace amcl3d
{
Node::Node() : grid3d_(), pf_(), nh_(ros::this_node::getName())
{
  ROS_DEBUG("[%s] Node::Node()", ros::this_node::getName().data());
}

Node::~Node()
{
  ROS_DEBUG("[%s] Node::~Node()", ros::this_node::getName().data());
}

void Node::spin()
{
  ROS_DEBUG("[%s] Node::spin()", ros::this_node::getName().data());

  if (!grid3d_.open(parameters_.map_path_, parameters_.sensor_dev_))
    return;

  if (parameters_.publish_grid_slice_rate_ != 0 &&
      grid3d_.buildGridSliceMsg(parameters_.grid_slice_z_, grid_slice_msg_))
  {
    grid_slice_msg_.header.frame_id = parameters_.global_frame_id_;
    grid_slice_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid_slice", 1, true);
    grid_slice_pub_timer_ =
        nh_.createTimer(ros::Duration(ros::Rate(parameters_.publish_grid_slice_rate_)), &Node::publishGridSlice, this);
  }

  if (parameters_.publish_point_cloud_rate_ != 0 && grid3d_.buildMapPointCloudMsg(map_point_cloud_msg_))
  {
    map_point_cloud_msg_.header.frame_id = parameters_.global_frame_id_;
    map_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_point_cloud", 1, true);
    map_point_cloud_pub_timer_ = nh_.createTimer(ros::Duration(ros::Rate(parameters_.publish_point_cloud_rate_)),
                                                 &Node::publishMapPointCloud, this);
  }

  point_sub_ = nh_.subscribe("/laser_sensor", 1, &Node::pointcloudCallback, this);
  odom_sub_ = nh_.subscribe("/odometry", 1, &Node::odomCallback, this);
  range_sub_ = nh_.subscribe("/radiorange_sensor", 1, &Node::rangeCallback, this);

  particles_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_cloud", 1, true);
  range_markers_pub_ = nh_.advertise<visualization_msgs::Marker>("range", 0);
  odom_base_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("base_transform", 1);

  cloud_filter_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_filtered", 0);

  while (ros::ok())
  {
    ros::spinOnce();
    usleep(100);
  }

  nh_.shutdown();
}

void Node::publishMapPointCloud(const ros::TimerEvent&)
{
  ROS_DEBUG("[%s] Node::publishMapPointCloud()", ros::this_node::getName().data());

  map_point_cloud_msg_.header.stamp = ros::Time::now();
  map_point_cloud_pub_.publish(map_point_cloud_msg_);
}

void Node::publishGridSlice(const ros::TimerEvent&)
{
  ROS_DEBUG("[%s] Node::publishGridSlice()", ros::this_node::getName().data());

  grid_slice_msg_.header.stamp = ros::Time::now();
  grid_slice_pub_.publish(grid_slice_msg_);
}

void Node::publishParticles()
{
  //! If the filter is not initialized then exit
  if (!pf_.isInitialized())
    return;

  //! Build the msg based on the particles position and orientation
  geometry_msgs::PoseArray msg;
  pf_.buildParticlesPoseMsg(msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = parameters_.global_frame_id_;

  //! Publish particle cloud
  particles_pose_pub_.publish(msg);
}

void Node::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  ROS_INFO("pointcloudCallback open");

  if (!is_odom_)
  {
    ROS_WARN("Odometry transform not received");
    return;
  }

  //! Check if an update must be performed or not
  if (!checkUpdateThresholds())
    return;

  static const ros::Duration update_interval(1.0 / parameters_.update_rate_);
  nextupdate_time_ = ros::Time::now() + update_interval;

  //! Apply voxel grid
  clock_t begin_filter = clock();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud_src);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_src);
  sor.setLeafSize(parameters_.voxel_size_, parameters_.voxel_size_, parameters_.voxel_size_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter(*cloud_down);
  cloud_down->header = cloud_src->header;
  sensor_msgs::PointCloud2 cloud_down_msg;
  pcl::toROSMsg(*cloud_down, cloud_down_msg);
  cloud_filter_pub_.publish(cloud_down_msg);
  clock_t end_filter = clock();
  double elapsed_secs = double(end_filter - begin_filter) / CLOCKS_PER_SEC;
  ROS_INFO("Filter time: [%lf] sec", elapsed_secs);

  //! Perform particle prediction based on odometry
  odom_increment_tf_ = lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;
  const double delta_x = odom_increment_tf_.getOrigin().getX();
  const double delta_y = odom_increment_tf_.getOrigin().getY();
  const double delta_z = odom_increment_tf_.getOrigin().getZ();
  const double delta_a = getYawFromTf(odom_increment_tf_);

  clock_t begin_predict = clock();
  pf_.predict(parameters_.odom_x_mod_, parameters_.odom_y_mod_, parameters_.odom_z_mod_, parameters_.odom_a_mod_,
              delta_x, delta_y, delta_z, delta_a);
  clock_t end_predict = clock();
  elapsed_secs = double(end_predict - begin_predict) / CLOCKS_PER_SEC;
  ROS_INFO("Predict time: [%lf] sec", elapsed_secs);

  //! Perform particle update based on current point-cloud
  clock_t begin_update = clock();
  pf_.update(grid3d_, cloud_down, range_data, parameters_.alpha_, parameters_.sensor_range_);
  clock_t end_update = clock();
  elapsed_secs = double(end_update - begin_update) / CLOCKS_PER_SEC;
  ROS_INFO("Update time: [%lf] sec", elapsed_secs);

  mean_p_ = pf_.getMean();

  //! Clean the range buffer
  range_data.clear();

  //! Update time and transform information
  lastupdatebase_2_odom_tf_ = base_2_odom_tf_;

  //! Do the resampling if needed
  clock_t begin_resample = clock();
  static int n_updates = 0;
  if (++n_updates > parameters_.resample_interval_)
  {
    n_updates = 0;
    pf_.resample();
  }
  clock_t end_resample = clock();
  elapsed_secs = double(end_resample - begin_resample) / CLOCKS_PER_SEC;
  ROS_INFO("Resample time: [%lf] sec", elapsed_secs);

  //! Publish particles
  publishParticles();

  ROS_INFO("pointcloudCallback close");
}

void Node::odomCallback(const geometry_msgs::TransformStampedConstPtr& msg)
{
  ROS_INFO("odomCallback open");

  base_2_odom_tf_.setOrigin(
      tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
  base_2_odom_tf_.setRotation(tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y,
                                             msg->transform.rotation.z, msg->transform.rotation.w));

  //! If the filter is not initialized then exit
  if (!pf_.isInitialized())
  {
    ROS_WARN("Filter not initialized yet, waiting for initial pose.");
    if (parameters_.set_initial_pose_)
    {
      tf::Transform init_pose;
      init_pose.setOrigin(tf::Vector3(parameters_.init_x_, parameters_.init_y_, parameters_.init_z_));
      init_pose.setRotation(tf::Quaternion(0.0, 0.0, sin(parameters_.init_a_ * 0.5), cos(parameters_.init_a_ * 0.5)));
      setInitialPose(init_pose, parameters_.init_x_dev_, parameters_.init_y_dev_, parameters_.init_z_dev_,
                     parameters_.init_a_dev_);
    }
    return;
  }

  //! Update roll and pitch from odometry
  double yaw;
  base_2_odom_tf_.getBasis().getRPY(roll_, pitch_, yaw);

  static tf::TransformBroadcaster tf_br;
  tf_br.sendTransform(
      tf::StampedTransform(base_2_odom_tf_, ros::Time::now(), parameters_.odom_frame_id_, parameters_.base_frame_id_));

  if (!is_odom_)
  {
    is_odom_ = true;

    lastbase_2_world_tf_ = initodom_2_world_tf_ * base_2_odom_tf_;
    lastodom_2_world_tf_ = initodom_2_world_tf_;
  }

  static bool has_takenoff = false;
  if (!has_takenoff)
  {
    ROS_WARN("Not <<taken off>> yet");

    //! Check takeoff height
    has_takenoff = base_2_odom_tf_.getOrigin().getZ() > parameters_.take_off_height_;

    lastbase_2_world_tf_ = initodom_2_world_tf_ * base_2_odom_tf_;
    lastodom_2_world_tf_ = initodom_2_world_tf_;

    lastmean_p_ = mean_p_;  // for not 'jumping' whenever has_takenoff is true
  }
  else
  {
    //! Check if AMCL went wrong (nan, inf)
    if (std::isnan(mean_p_.x) || std::isnan(mean_p_.y) || std::isnan(mean_p_.z) || std::isnan(mean_p_.a))
    {
      ROS_WARN("AMCL NaN detected");
      amcl_out_ = true;
    }
    if (std::isinf(mean_p_.x) || std::isinf(mean_p_.y) || std::isinf(mean_p_.z) || std::isinf(mean_p_.a))
    {
      ROS_WARN("AMCL Inf detected");
      amcl_out_ = true;
    }

    //! Check jumps
    if (fabs(mean_p_.x - lastmean_p_.x) > 1.)
    {
      ROS_WARN("AMCL Jump detected in X");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.y - lastmean_p_.y) > 1.)
    {
      ROS_WARN("AMCL Jump detected in Y");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.z - lastmean_p_.z) > 1.)
    {
      ROS_WARN("AMCL Jump detected in Z");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.a - lastmean_p_.a) > 1.)
    {
      ROS_WARN("AMCL Jump detected in Yaw");
      amcl_out_ = true;
    }

    if (!amcl_out_)
    {
      tf::Transform base_2_world_tf;
      base_2_world_tf.setOrigin(tf::Vector3(mean_p_.x, mean_p_.y, mean_p_.z));
      tf::Quaternion q;
      q.setRPY(roll_, pitch_, mean_p_.a);
      base_2_world_tf.setRotation(q);

      base_2_world_tf = base_2_world_tf * lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;

      lastmean_p_ = mean_p_;

      lastbase_2_world_tf_ = base_2_world_tf;
      lastodom_2_world_tf_ = base_2_world_tf * base_2_odom_tf_.inverse();

      amcl_out_lastbase_2_odom_tf_ = lastupdatebase_2_odom_tf_;
    }
    else
    {
      lastbase_2_world_tf_ = lastbase_2_world_tf_ * amcl_out_lastbase_2_odom_tf_.inverse() * base_2_odom_tf_;
      amcl_out_lastbase_2_odom_tf_ = base_2_odom_tf_;
    }
  }

  //! Publish transform
  geometry_msgs::TransformStamped odom_2_base_tf;
  odom_2_base_tf.header.stamp = msg->header.stamp;
  odom_2_base_tf.header.frame_id = parameters_.global_frame_id_;
  odom_2_base_tf.child_frame_id = parameters_.base_frame_id_;
  odom_2_base_tf.transform.translation.x = lastbase_2_world_tf_.getOrigin().getX();
  odom_2_base_tf.transform.translation.y = lastbase_2_world_tf_.getOrigin().getY();
  odom_2_base_tf.transform.translation.z = lastbase_2_world_tf_.getOrigin().getZ();
  odom_2_base_tf.transform.rotation.x = lastbase_2_world_tf_.getRotation().getX();
  odom_2_base_tf.transform.rotation.y = lastbase_2_world_tf_.getRotation().getY();
  odom_2_base_tf.transform.rotation.z = lastbase_2_world_tf_.getRotation().getZ();
  odom_2_base_tf.transform.rotation.w = lastbase_2_world_tf_.getRotation().getW();
  odom_base_pub_.publish(odom_2_base_tf);

  tf_br.sendTransform(tf::StampedTransform(lastodom_2_world_tf_, ros::Time::now(), parameters_.global_frame_id_,
                                           parameters_.odom_frame_id_));

  ROS_INFO("odomCallback close");
}

void Node::rangeCallback(const rosinrange_msg::RangePoseConstPtr& msg)
{
  ROS_INFO("rangeCallback open");

  geometry_msgs::Point anchor;
  anchor.x = msg->position.x;
  anchor.y = msg->position.y;
  anchor.z = msg->position.z;

  range_data.push_back(Range(static_cast<float>(msg->range), msg->position.x, msg->position.y, msg->position.z));

  geometry_msgs::Point uav;
  uav.x = mean_p_.x;
  uav.y = mean_p_.y;
  uav.z = mean_p_.z;

  rvizMarkerPublish(msg->source_id, static_cast<float>(msg->range), uav, anchor);

  ROS_INFO("rangeCallback close");
}

bool Node::checkUpdateThresholds()
{
  ROS_DEBUG("Checking for AMCL3D update");

  if (ros::Time::now() < nextupdate_time_)
    return false;

  odom_increment_tf_ = lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;

  //! Check translation threshold
  if (odom_increment_tf_.getOrigin().length() > parameters_.d_th_)
  {
    ROS_INFO("Translation update");
    return true;
  }

  //! Check yaw threshold
  double yaw, pitch, roll;
  odom_increment_tf_.getBasis().getRPY(roll, pitch, yaw);
  if (fabs(yaw) > parameters_.a_th_)
  {
    ROS_INFO("Rotation update");
    return true;
  }

  return false;
}

void Node::setInitialPose(const tf::Transform& init_pose, const float x_dev, const float y_dev, const float z_dev,
                          const float a_dev)
{
  initodom_2_world_tf_ = init_pose;

  const tf::Vector3 t = init_pose.getOrigin();

  const float x_init = t.x();
  const float y_init = t.y();
  const float z_init = t.z();
  const float a_init = static_cast<float>(getYawFromTf(init_pose));

  pf_.init(parameters_.num_particles_, x_init, y_init, z_init, a_init, x_dev, y_dev, z_dev, a_dev);

  mean_p_ = pf_.getMean();
  lastmean_p_ = mean_p_;

  //! Extract TFs for future updates
  //! Reset lastupdatebase_2_odom_tf_
  lastupdatebase_2_odom_tf_ = base_2_odom_tf_;

  //! Publish particles
  publishParticles();
}

double Node::getYawFromTf(const tf::Transform& tf)
{
  double yaw, pitch, roll;
  tf.getBasis().getRPY(roll, pitch, yaw);

  return yaw;
}

void Node::rvizMarkerPublish(const uint32_t anchor_id, const float r, const geometry_msgs::Point& uav,
                             const geometry_msgs::Point& anchor)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = parameters_.global_frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "amcl3d";
  marker.id = anchor_id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = r;
  marker.scale.z = r;
  marker.color.a = 0.5;
  if (amcl_out_)  //! Indicate if AMCL was lost
  {
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
  }
  else
  {
    switch (anchor_id)
    {
      case 1:
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        break;
      case 2:
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        break;
      case 3:
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        break;
    }
  }
  marker.points.clear();
  marker.points.push_back(uav);
  marker.points.push_back(anchor);

  //! Publish marker
  range_markers_pub_.publish(marker);
}

}  // namespace amcl3d
