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
