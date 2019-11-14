/*!
 * @file Grid3d.h
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

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

#include "PointCloudTools.h"

namespace amcl3d
{
class Grid3d
{
public:
  explicit Grid3d()
  {
  }
  virtual ~Grid3d()
  {
  }

  bool open(const std::string& map_path, const double sensor_dev);

  bool buildGridSliceMsg(const double z, nav_msgs::OccupancyGrid& msg) const;
  bool buildMapPointCloudMsg(sensor_msgs::PointCloud2& msg) const;

  float computeCloudWeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const float tx, const float ty,
                           const float tz, const float a) const;
  bool isIntoMap(const float x, const float y, const float z) const;

private:
  inline uint32_t point2grid(const float x, const float y, const float z) const;

  //! 3D point cloud representation of the map
  PointCloudInfo::Ptr pc_info_;

  //! 3D probabilistic grid cell
  Grid3dInfo::Ptr grid_info_;
};

}  // namespace amcl3d
