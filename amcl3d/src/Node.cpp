#include "Node.h"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>

namespace amcl3d
{
Node::Node() : grid3d_(parameters_.sensor_dev), pf_(), nh_(ros::this_node::getName())
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

  if (!grid3d_.open(parameters_.map_path))
    return;

  if (parameters_.publish_grid_slice_rate != 0 && grid3d_.buildGridSliceMsg(parameters_.grid_slice, grid_slice_msg_))
  {
    grid_slice_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid_slice", 1, true);
    grid_slice_pub_timer_ =
        nh_.createTimer(ros::Duration(ros::Rate(parameters_.publish_grid_slice_rate)), &Node::publishGridSlice, this);
  }

  if (parameters_.publish_point_cloud_rate != 0 && grid3d_.buildMapPointCloudMsg(map_point_cloud_msg_))
  {
    map_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_point_cloud", 1, true);
    map_point_cloud_pub_timer_ = nh_.createTimer(ros::Duration(ros::Rate(parameters_.publish_point_cloud_rate)),
                                                 &Node::publishMapPointCloud, this);
  }

  grid3d_.buildGrid3d2WorldTf(parameters_.globalFrameId_, grid_to_world_tf_);

  if (parameters_.publish_grid_tf_rate != 0)
  {
    grid_to_world_tf_timer_ =
        nh_.createTimer(ros::Duration(ros::Rate(parameters_.publish_grid_tf_rate)), &Node::publishGridTf, this);
  }

  point_sub_ = nh_.subscribe(parameters_.inCloudTopic, 1, &Node::pointcloudCallback, this);
  odom_sub_ = nh_.subscribe(parameters_.inOdomTopic, 1, &Node::odomCallback, this);
  range_sub_ = nh_.subscribe(parameters_.inRangeTopic, 1, &Node::rangeCallback, this);
  initialpose_sub_ = nh_.subscribe("initial_pose", 2, &Node::initialPoseReceived, this);

  particles_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_cloud", 1, true);
  range_markers_pub_ = nh_.advertise<visualization_msgs::Marker>("range", 0);
  odom_base_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("base_transform", 1);

  cloud_filter_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_voxelfiltered", 0);
  cloud_passfilter_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_passfiltered", 0);

  while (ros::ok())
  {
    ros::spinOnce();
    usleep(10000);
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

void Node::publishGridTf(const ros::TimerEvent&)
{
  ROS_DEBUG("[%s] Node::publishGridSlice()", ros::this_node::getName().data());

  grid_to_world_tf_.stamp_ = ros::Time::now();

  tf::TransformBroadcaster tf_broadcaster_;
  tf_broadcaster_.sendTransform(grid_to_world_tf_);
}

void Node::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (!is_odom_)
  {
    ROS_WARN("Odometry transform not received");
    return;
  }

  //! If the filter is not initialized then exit
  if (!pf_.isInitialized())
  {
    ROS_WARN("Filter not initialized yet, waiting for initial pose.");
    if (parameters_.setInitialPose_)
    {
      tf::Transform initPose;
      initPose.setOrigin(tf::Vector3(parameters_.initX_, parameters_.initY_, parameters_.initZ_));
      initPose.setRotation(tf::Quaternion(0.0, 0.0, sin(parameters_.initA_ * 0.5), cos(parameters_.initA_ * 0.5)));
      setInitialPose(initPose, parameters_.initXDev_, parameters_.initYDev_, parameters_.initZDev_,
                     parameters_.initADev_);
    }
    return;
  }

  //! Check if an update must be performed or not
  if (!checkUpdateThresholds())
    return;

  static const ros::Duration updateInterval(1.0 / parameters_.updateRate_);
  nextupdate_time_ = ros::Time::now() + updateInterval;

  //! Apply pass-though and voxel grid
  clock_t begin_filter = clock();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-10, 0);
  pass.setNegative(false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter(*cloud_filtered);
  cloud_filtered->header = cloud->header;
  sensor_msgs::PointCloud2 passCloud;
  pcl::toROSMsg(*cloud_filtered, passCloud);
  cloud_passfilter_pub_.publish(passCloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(passCloud, *cloud_src);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_src);
  sor.setLeafSize(parameters_.voxelSize_, parameters_.voxelSize_, parameters_.voxelSize_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter(*cloud_down);
  cloud_down->header = cloud_src->header;
  sensor_msgs::PointCloud2 downCloud;
  pcl::toROSMsg(*cloud_down, downCloud);
  cloud_filter_pub_.publish(downCloud);
  clock_t end_filter = clock();
  double elapsed_secs = double(end_filter - begin_filter) / CLOCKS_PER_SEC;
  ROS_INFO("Filter time: [%lf] sec", elapsed_secs);

  //! Perform particle prediction based on odometry
  odom_increment_tf = lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;
  const double delta_x = odom_increment_tf.getOrigin().getX();
  const double delta_y = odom_increment_tf.getOrigin().getY();
  const double delta_z = odom_increment_tf.getOrigin().getZ();
  const double delta_a = getYawFromTf(odom_increment_tf);

  clock_t begin_predict = clock();
  pf_.predict(parameters_.odomXMod_, parameters_.odomYMod_, parameters_.odomZMod_, parameters_.odomAMod_, delta_x,
              delta_y, delta_z, delta_a);
  clock_t end_predict = clock();
  elapsed_secs = double(end_predict - begin_predict) / CLOCKS_PER_SEC;
  ROS_INFO("Predict time: [%lf] sec", elapsed_secs);

  //! Compensate for the current roll and pitch of the base-link
  const float sr = sin(roll_);
  const float cr = cos(roll_);
  const float sp = sin(pitch_);
  const float cp = cos(pitch_);
  float r00, r01, r02, r10, r11, r12, r20, r21, r22;
  r00 = cp;
  r01 = sp * sr;
  r02 = cr * sp;
  r10 = 0;
  r11 = cr;
  r12 = -sr;
  r20 = -sp;
  r21 = cp * sr;
  r22 = cp * cr;
  sensor_msgs::PointCloud2ConstIterator<float> iterX(downCloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iterY(downCloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iterZ(downCloud, "z");
  std::vector<pcl::PointXYZ> points;
  points.resize(downCloud.width);
  for (uint32_t i = 0; i < downCloud.width; ++i, ++iterX, ++iterY, ++iterZ)
  {
    points[i].x = *iterX * r00 + *iterY * r01 + *iterZ * r02;
    points[i].y = *iterX * r10 + *iterY * r11 + *iterZ * r12;
    points[i].z = *iterX * r20 + *iterY * r21 + *iterZ * r22;
  }

  //! Perform particle update based on current point-cloud
  clock_t begin_update = clock();
  pf_.update(grid3d_, points, range_data, parameters_.alpha_, parameters_.sensor_range);
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
  if (++n_updates > parameters_.resampleInterval_)
  {
    n_updates = 0;
    pf_.resample();
  }
  clock_t end_resample = clock();
  elapsed_secs = double(end_resample - begin_resample) / CLOCKS_PER_SEC;
  ROS_INFO("Resample time: [%lf] sec", elapsed_secs);

  //! Publish particles
  publishParticles();
}

void Node::odomCallback(const geometry_msgs::TransformStampedConstPtr& msg)
{
  base_2_odom_tf_.setOrigin(
      tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
  base_2_odom_tf_.setRotation(tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y,
                                             msg->transform.rotation.z, msg->transform.rotation.w));

  //! Update roll and pitch from odometry
  double yaw;
  base_2_odom_tf_.getBasis().getRPY(roll_, pitch_, yaw);

  static tf::TransformBroadcaster tfBr;
  tfBr.sendTransform(
      tf::StampedTransform(base_2_odom_tf_, ros::Time::now(), parameters_.odomFrameId_, parameters_.baseFrameId_));

  if (!is_odom_)
  {
    is_odom_ = true;

    lastbase_2_world_tf_ = initodom_2_world_tf_ * base_2_odom_tf_;
    lastodom_2_world_tf_ = initodom_2_world_tf_;
  }

  if (!pf_.isInitialized())
  {
    ROS_WARN("Filter not initialized yet, not publishing output TF");
    return;
  }

  static bool has_takenoff = false;
  if (!has_takenoff)
  {
    ROS_WARN("Not <<taken off>> yet");

    //! Check takeoff height
    has_takenoff = base_2_odom_tf_.getOrigin().getZ() > parameters_.takeOffHeight_;

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
      ROS_WARN_STREAM("AMCL Jump detected in X");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.y - lastmean_p_.y) > 1.)
    {
      ROS_WARN_STREAM("AMCL Jump detected in Y");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.z - lastmean_p_.z) > 1.)
    {
      ROS_WARN_STREAM("AMCL Jump detected in Z");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.a - lastmean_p_.a) > 1.)
    {
      ROS_WARN_STREAM("AMCL Jump detected in Yaw");
      amcl_out_ = true;
    }

    if (!amcl_out_)
    {
      geometry_msgs::Point32 grid3d;
      grid3d_.getMinOctomap(grid3d.x, grid3d.y, grid3d.z);

      tf::Transform base_2_world_tf;
      base_2_world_tf.setOrigin(tf::Vector3(mean_p_.x + grid3d.x, mean_p_.y + grid3d.y, mean_p_.z + grid3d.z));
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
  odom_2_base_tf.header.frame_id = parameters_.globalFrameId_;
  odom_2_base_tf.child_frame_id = parameters_.baseFrameId_;
  odom_2_base_tf.transform.translation.x = lastbase_2_world_tf_.getOrigin().getX();
  odom_2_base_tf.transform.translation.y = lastbase_2_world_tf_.getOrigin().getY();
  odom_2_base_tf.transform.translation.z = lastbase_2_world_tf_.getOrigin().getZ();
  odom_2_base_tf.transform.rotation.x = lastbase_2_world_tf_.getRotation().getX();
  odom_2_base_tf.transform.rotation.y = lastbase_2_world_tf_.getRotation().getY();
  odom_2_base_tf.transform.rotation.z = lastbase_2_world_tf_.getRotation().getZ();
  odom_2_base_tf.transform.rotation.w = lastbase_2_world_tf_.getRotation().getW();
  odom_base_pub_.publish(odom_2_base_tf);

  tfBr.sendTransform(tf::StampedTransform(lastodom_2_world_tf_, ros::Time::now(), parameters_.globalFrameId_,
                                          parameters_.odomFrameId_));
}

void Node::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  //! We only accept initial pose estimates in the global frame
  if (msg->header.frame_id != parameters_.globalFrameId_)
  {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; "
             "initial poses must be in the global frame, \"%s\"",
             msg->header.frame_id.c_str(), parameters_.globalFrameId_.c_str());
    return;
  }

  //! Transform into the global frame
  tf::Transform pose;
  tf::poseMsgToTF(msg->pose.pose, pose);

  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f %.3f", ros::Time::now().toSec(), pose.getOrigin().x(),
           pose.getOrigin().y(), pose.getOrigin().z(), getYawFromTf(pose));

  //! Initialize the filter
  setInitialPose(pose, parameters_.initXDev_, parameters_.initYDev_, parameters_.initZDev_, parameters_.initADev_);
}

void Node::rangeCallback(const rosinrange_msg::range_poseConstPtr& msg)
{
  const int node = msg->destination_id;

  geometry_msgs::Point32 grid3d;
  grid3d_.getMinOctomap(grid3d.x, grid3d.y, grid3d.z);

  geometry_msgs::Point anchor;
  anchor.x = msg->position.x;
  anchor.y = msg->position.y;
  anchor.z = msg->position.z;

  float ax, ay, az;
  ax = msg->position.x - grid3d.x;
  ay = msg->position.y - grid3d.y;
  az = msg->position.z - grid3d.z;

  range_data.push_back(Range(static_cast<float>(msg->range), ax, ay, az));

  geometry_msgs::Point uav;
  uav.x = mean_p_.x + grid3d.x;
  uav.y = mean_p_.y + grid3d.y;
  uav.z = mean_p_.z + grid3d.z;

  RvizMarkerPublish(msg->destination_id, static_cast<float>(msg->range), uav, anchor);
}

bool Node::checkUpdateThresholds()
{
  std::cout << "Checking for AMCL3D update" << std::endl;

  if (ros::Time::now() < nextupdate_time_)
    return false;

  odom_increment_tf = lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;

  //! Check translation threshold
  if (odom_increment_tf.getOrigin().length() > parameters_.dTh_)
  {
    ROS_INFO("Translation update");
    return true;
  }

  //! Check yaw threshold
  double yaw, pitch, roll;
  odom_increment_tf.getBasis().getRPY(roll, pitch, yaw);
  if (fabs(yaw) > parameters_.aTh_)
  {
    ROS_INFO("Rotation update");
    return true;
  }

  return false;
}

void Node::publishParticles()
{
  //! If the filter is not initialized then exit
  if (!pf_.isInitialized())
    return;

  geometry_msgs::Point32 grid3d;
  grid3d_.getMinOctomap(grid3d.x, grid3d.y, grid3d.z);

  //! Build the msg based on the particles position and orinetation
  geometry_msgs::PoseArray msg;
  pf_.buildParticlesPoseMsg(grid3d, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = parameters_.globalFrameId_;

  //! Publish particle cloud
  particles_pose_pub_.publish(msg);
}

void Node::setInitialPose(const tf::Transform& init_pose, const float x_dev, const float y_dev, const float z_dev,
                          const float a_dev)
{
  initodom_2_world_tf_ = init_pose;

  geometry_msgs::Point32 grid3d;
  grid3d_.getMinOctomap(grid3d.x, grid3d.y, grid3d.z);

  const tf::Vector3 t = init_pose.getOrigin();

  const float xInit = t.x() - grid3d.x;
  const float yInit = t.y() - grid3d.y;
  const float zInit = t.z() - grid3d.z + parameters_.initZOffset_;
  const float aInit = static_cast<float>(getYawFromTf(init_pose));

  pf_.init(parameters_.num_particles, xInit, yInit, zInit, aInit, x_dev, y_dev, z_dev, a_dev);

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

void Node::RvizMarkerPublish(uint32_t anchor_id, float r, geometry_msgs::Point uav, geometry_msgs::Point anchor)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = parameters_.globalFrameId_;
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
      case 0:
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        break;
      case 1:
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        break;
      case 2:
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