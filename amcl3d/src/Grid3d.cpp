/*!
 * @file Grid3d.cpp
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

#include "Grid3d.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

namespace amcl3d
{
Grid3d::Grid3d(const double sensor_dev) : sensor_dev_(sensor_dev)
{
}

bool Grid3d::open(const std::string& map_path)
{
  try
  {
    //! Load octomap
    auto octo_tree = openOcTree(map_path);

    ROS_INFO("[%s] Octomap loaded", ros::this_node::getName().data());

    //! Compute the point-cloud associated to the octomap
    pc_info_ = computePointCloud(octo_tree);

    ROS_INFO("[%s]"
             "\n   Map size:"
             "\n      X: %lf to %lf"
             "\n      Y: %lf to %lf"
             "\n      Z: %lf to %lf"
             "\n      Res: %lf",
             ros::this_node::getName().data(),
             pc_info_.octo_min_x, pc_info_.octo_max_x,
             pc_info_.octo_min_y, pc_info_.octo_max_y,
             pc_info_.octo_min_z, pc_info_.octo_max_z,
             pc_info_.octo_resol);
  }
  catch (std::exception &e)
  {
    ROS_ERROR("[%s] %s", ros::this_node::getName().data(), e.what());
    return false;
  }

  //! Try to load the associated grid-map from file
  std::string grid_path;
  if (map_path.compare(map_path.length() - 3, 3, ".bt") == 0)
    grid_path = map_path.substr(0, map_path.find(".bt")) + ".grid";
  if (map_path.compare(map_path.length() - 3, 3, ".ot") == 0)
    grid_path = map_path.substr(0, map_path.find(".ot")) + ".grid";

  if (loadGrid(grid_path))
    return true;

  //! Compute the gridMap using kdtree search over the point-cloud
  ROS_INFO("[%s] Computing 3D occupancy grid. This will take some time...", ros::this_node::getName().data());
  grid_info_ = computeGrid(pc_info_, sensor_dev_);
  ROS_INFO("[%s] Computing 3D occupancy grid done!", ros::this_node::getName().data());

  //! Save grid on file
  saveGrid(grid_path);

  return true;
}

bool Grid3d::buildGridSliceMsg(const float z, nav_msgs::OccupancyGrid& msg) const
{
  if (!grid_info_.grid)
    return false;

  if (z < 0 || z > grid_info_.max_x)
    return false;

  msg.header.frame_id = "grid3d";
  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = pc_info_.octo_resol;
  msg.info.width = grid_info_.size_x;
  msg.info.height = grid_info_.size_y;
  msg.info.origin.position.x = 0.;
  msg.info.origin.position.y = 0.;
  msg.info.origin.position.z = static_cast<double>(z);
  msg.info.origin.orientation.x = 0.;
  msg.info.origin.orientation.y = 0.;
  msg.info.origin.orientation.z = 0.;
  msg.info.origin.orientation.w = 1.;

  //! Extract max probability
  const uint32_t init = point2grid(0, 0, z);
  const uint32_t end  = point2grid(grid_info_.max_x, grid_info_.max_y, z);
  float temp_prob, max_prob = -1.0;
  for (uint32_t i = init; i < end; ++i)
  {
    temp_prob = grid_info_.grid[i].prob;
    if (temp_prob > max_prob)
      max_prob = temp_prob;
  }

  //! Copy data into grid msg and scale the probability to [0, 100]
  if (max_prob < 0.000001f)
    max_prob = 0.000001f;
  max_prob = 100.f / max_prob;
  msg.data.resize(end - init);
  for (uint32_t i = 0; i < msg.data.size(); ++i)
    msg.data[i] = static_cast<int8_t>(grid_info_.grid[init + i].prob * max_prob);

  return true;
}

bool Grid3d::buildMapPointCloudMsg(sensor_msgs::PointCloud2& msg) const
{
  if (!pc_info_.cloud)
    return false;

  pcl::toROSMsg(*pc_info_.cloud, msg);

  msg.header.frame_id = "grid3d";

  return true;
}

void Grid3d::buildGrid3d2WorldTf(const std::string& global_frame_id, tf::StampedTransform& tf) const
{
  tf::Transform grid3d_2_world_tf;
  grid3d_2_world_tf.setOrigin(tf::Vector3(pc_info_.octo_min_x, pc_info_.octo_min_y, pc_info_.octo_min_z));
  grid3d_2_world_tf.setRotation(tf::Quaternion(0, 0, 0, 1));

  tf = tf::StampedTransform(grid3d_2_world_tf, ros::Time::now(), global_frame_id, "grid3d");
}

float Grid3d::computeCloudWeight(const std::vector<pcl::PointXYZ>& points, const float tx, const float ty,
                                 const float tz, const float a) const
{
  float weight = 0.;
  int n = 0;

  pcl::PointXYZ new_point;

  const auto sa = sin(a);
  const auto ca = cos(a);

  if (!grid_info_.grid)
    return 0;

  for (uint32_t i = 0; i < points.size(); ++i)
  {
    const auto& p = points[i];

    new_point.x = ca * p.x - sa * p.y + tx;
    new_point.y = sa * p.x + ca * p.y + ty;
    new_point.z = p.z + tz;

    if (new_point.x >= 0.f && new_point.x < grid_info_.max_x &&
        new_point.y >= 0.f && new_point.y < grid_info_.max_y &&
        new_point.z >= 0.f && new_point.z < grid_info_.max_z)
    {
      weight += grid_info_.grid[point2grid(new_point.x, new_point.y, new_point.z)].prob;
      n += 1;
    }
  }
  return (n <= 10) ? 0 : weight / n;
}

bool Grid3d::isIntoMap(const float x, const float y, const float z) const
{
  return (x >= 0.f && x <= grid_info_.max_x &&
          y >= 0.f && y <= grid_info_.max_y &&
          z >= 0.f && z <= grid_info_.max_z);
}

void Grid3d::getMinOctomap(float& x, float& y, float& z) const
{
  x = pc_info_.octo_min_x;
  y = pc_info_.octo_min_y;
  z = pc_info_.octo_min_z;
}

bool Grid3d::saveGrid(const std::string& grid_path)
{
  if (!grid_info_.grid)
    return false;

  auto pf = fopen(grid_path.c_str(), "wb");
  if (!pf)
  {
    ROS_ERROR("[%s] Error opening file %s for writing", ros::this_node::getName().data(), grid_path.c_str());
    return false;
  }

  //! Write grid general info
  fwrite(&grid_info_.size, sizeof(uint32_t), 1, pf);
  fwrite(&grid_info_.size_x, sizeof(uint32_t), 1, pf);
  fwrite(&grid_info_.size_y, sizeof(uint32_t), 1, pf);
  fwrite(&grid_info_.size_z, sizeof(uint32_t), 1, pf);
  fwrite(&sensor_dev_, sizeof(double), 1, pf);

  //! Write grid cells
  fwrite(grid_info_.grid.get(), sizeof(Grid3dCell), grid_info_.size, pf);

  fclose(pf);

  ROS_INFO("[%s] Grid map successfully saved on %s", ros::this_node::getName().data(), grid_path.c_str());

  return true;
}

bool Grid3d::loadGrid(const std::string& grid_path)
{
  auto pf = fopen(grid_path.c_str(), "rb");
  if (!pf)
  {
    ROS_ERROR("[%s] Error opening file %s for reading", ros::this_node::getName().data(), grid_path.c_str());
    return false;
  }

  //! Read grid general info
  double sensor_dev;
  fread(&grid_info_.size, sizeof(uint32_t), 1, pf);
  fread(&grid_info_.size_x, sizeof(uint32_t), 1, pf);
  fread(&grid_info_.size_y, sizeof(uint32_t), 1, pf);
  fread(&grid_info_.size_z, sizeof(uint32_t), 1, pf);
  fread(&sensor_dev, sizeof(double), 1, pf);

  if (std::fabs(sensor_dev - sensor_dev_) >= std::numeric_limits<double>::epsilon())
  {
    ROS_ERROR("[%s] Loaded sensorDev is different", ros::this_node::getName().data());
    return false;
  }

  grid_info_.step_y = grid_info_.size_x;
  grid_info_.step_z = grid_info_.size_x * grid_info_.size_y;

  //! Read grid cells
  grid_info_.grid.reset(new Grid3dCell[grid_info_.size]);
  fread(grid_info_.grid.get(), sizeof(Grid3dCell), grid_info_.size, pf);

  fclose(pf);

  ROS_INFO("[%s] Grid map successfully loaded from %s", ros::this_node::getName().data(), grid_path.c_str());

  return true;
}

inline uint32_t Grid3d::point2grid(const float x, const float y, const float z) const
{
  return static_cast<uint32_t>(x / pc_info_.octo_resol) +
         static_cast<uint32_t>(y / pc_info_.octo_resol) * grid_info_.step_y +
         static_cast<uint32_t>(z / pc_info_.octo_resol) * grid_info_.step_z;
}

}  // namespace amcl3d
