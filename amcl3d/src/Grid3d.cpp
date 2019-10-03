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

#ifdef _OPENMP
#include <omp.h>
#endif

namespace amcl3d
{
Grid3d::Grid3d(const double sensor_dev) : sensor_dev_(sensor_dev)
{
}

Grid3d::~Grid3d()
{
}

bool Grid3d::open(const std::string& map_path)
{
  //! Load octomap
  if (!loadOctomap(map_path))
    return false;

  //! Compute the point-cloud associated to the octomap
  computePointCloud();

  //! Try to load the associated grid-map from file
  std::string grid_path;
  if (map_path.compare(map_path.length() - 3, 3, ".bt") == 0)
    grid_path = map_path.substr(0, map_path.find(".bt")) + ".grid";
  if (map_path.compare(map_path.length() - 3, 3, ".ot") == 0)
    grid_path = map_path.substr(0, map_path.find(".ot")) + ".grid";

  if (loadGrid(grid_path))
    return true;

  //! Compute the gridMap using kdtree search over the point-cloud
  computeGrid();

  //! Save grid on file
  saveGrid(grid_path);

  return true;
}

bool Grid3d::buildGridSliceMsg(const float z, nav_msgs::OccupancyGrid& msg) const
{
  if (!grid_)
    return false;

  if (z < 0 || z > max_z_)
    return false;

  msg.header.frame_id = "grid3d";
  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = resolution_;
  msg.info.width = grid_size_x_;
  msg.info.height = grid_size_y_;
  msg.info.origin.position.x = 0.;
  msg.info.origin.position.y = 0.;
  msg.info.origin.position.z = static_cast<double>(z);
  msg.info.origin.orientation.x = 0.;
  msg.info.origin.orientation.y = 0.;
  msg.info.origin.orientation.z = 0.;
  msg.info.origin.orientation.w = 1.;
  msg.data.resize(grid_size_x_ * grid_size_y_);

  //! Extract max probability
  const uint32_t offset = static_cast<uint32_t>(z * one_div_res_) * grid_size_x_ * grid_size_y_;
  const uint32_t end = offset + grid_size_x_ * grid_size_y_;
  float max_prob = -1.0;
  for (uint32_t i = offset; i < end; ++i)
    if (grid_[i].prob > max_prob)
      max_prob = grid_[i].prob;

  //! Copy data into grid msg and scale the probability to [0, 100]
  if (max_prob < 0.000001f)
    max_prob = 0.000001f;
  max_prob = 100.f / max_prob;
  for (uint32_t i = 0; i < grid_size_x_ * grid_size_y_; ++i)
    msg.data[i] = static_cast<int8_t>(grid_[i + offset].prob * max_prob);

  return true;
}

bool Grid3d::buildMapPointCloudMsg(sensor_msgs::PointCloud2& msg) const
{
  if (!cloud_)
    return false;

  pcl::toROSMsg(*cloud_, msg);

  msg.header.frame_id = "grid3d";

  return true;
}

void Grid3d::buildGrid3d2WorldTf(const std::string& global_frame_id, tf::StampedTransform& tf) const
{
  tf::Transform grid3d_2_world_tf;
  grid3d_2_world_tf.setOrigin(tf::Vector3(min_oct_x_, min_oct_y_, min_oct_z_));
  grid3d_2_world_tf.setRotation(tf::Quaternion(0, 0, 0, 1));

  tf = tf::StampedTransform(grid3d_2_world_tf, ros::Time::now(), global_frame_id, "grid3d");
}

float Grid3d::computeCloudWeight(const std::vector<pcl::PointXYZ>& points, const float tx, const float ty,
                                 const float tz, const float a) const
{
  float total_weight = 0;
  int total_n = 0;

  const auto sa = sin(a);
  const auto ca = cos(a);
  std::vector<float> weight;

#pragma omp parallel
  {
    std::vector<float> weight_p;
#pragma omp for
    for (uint32_t i = 0; i < points.size(); ++i)
    {
      pcl::PointXYZ new_point;
      const auto& p = points[i];

      new_point.x = ca * p.x - sa * p.y + tx;
      new_point.y = sa * p.x + ca * p.y + ty;
      new_point.z = p.z + tz;

      if (new_point.x >= 0.f && new_point.y >= 0.f && new_point.z >= 0.f && new_point.x < max_x_ &&
          new_point.y < max_y_ && new_point.z < max_z_)
      {
        weight_p.push_back(grid_[point2grid(new_point.x, new_point.y, new_point.z)].prob);
      }
    }

#pragma omp critical
    {
      weight = weight_p;
      for (int i = 0; i < weight_p.size(); ++i)
      {
        total_weight += weight[i];
        total_n += 1;
      }
    }
  }

  return (total_n <= 10) ? 0 : total_weight / total_n;
}

bool Grid3d::isIntoMap(const float x, const float y, const float z) const
{
  return (x >= 0.f && y >= 0.f && z >= 0.f && x <= max_x_ && y <= max_y_ && z <= max_z_);
}

void Grid3d::getMinOctomap(float& x, float& y, float& z) const
{
  x = min_oct_x_;
  y = min_oct_y_;
  z = min_oct_z_;
}

bool Grid3d::loadOctomap(const std::string& map_path)
{
  if (map_path.length() <= 3)
    return false;

  if (map_path.compare(map_path.length() - 3, 3, ".bt") == 0)
  {
    octomap_.reset(new octomap::OcTree(0.1));
  }
  else if (map_path.compare(map_path.length() - 3, 3, ".ot") == 0)
  {
    octomap_.reset(dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::read(map_path)));
  }

  if (!octomap_)
  {
    ROS_ERROR("[%s] Error: NULL octomap!!", ros::this_node::getName().data());
    return false;
  }

  if (!octomap_->readBinary(map_path) || octomap_->size() <= 1)
    return false;

  ROS_INFO("[%s] Octomap loaded", ros::this_node::getName().data());

  // Get map parameters
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octomap_->getMetricMin(min_x, min_y, min_z);
  octomap_->getMetricMax(max_x, max_y, max_z);
  max_x_ = static_cast<float>(max_x - min_x);
  max_y_ = static_cast<float>(max_y - min_y);
  max_z_ = static_cast<float>(max_z - min_z);
  min_oct_x_ = static_cast<float>(min_x);
  min_oct_y_ = static_cast<float>(min_y);
  min_oct_z_ = static_cast<float>(min_z);
  resolution_ = static_cast<float>(octomap_->getResolution());
  one_div_res_ = 1.0f / resolution_;

  ROS_INFO("[%s]"
           "\n   Map size:"
           "\n      X: %lf to %lf"
           "\n      Y: %lf to %lf"
           "\n      Z: %lf to %lf"
           "\n      Res: %lf",
           ros::this_node::getName().data(), min_x, max_x, min_y, max_y, min_z, max_z, resolution_);

  return true;
}

void Grid3d::computePointCloud()
{
  //! Load the octomap in PCL for easy nearest neighborhood computation
  //! The point-cloud is shifted to have (0,0,0) as min values
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  cloud_->width = static_cast<uint32_t>(octomap_->size());
  cloud_->height = 1;
  cloud_->points.resize(cloud_->width * cloud_->height);

  uint32_t i = 0;
  for (octomap::OcTree::leaf_iterator it = octomap_->begin_leafs(); it != octomap_->end_leafs(); ++it)
  {
    if (it != nullptr && octomap_->isNodeOccupied(*it))
    {
      cloud_->points[i].x = static_cast<float>(it.getX()) - min_oct_x_;
      cloud_->points[i].y = static_cast<float>(it.getY()) - min_oct_y_;
      cloud_->points[i].z = static_cast<float>(it.getZ()) - min_oct_z_;
      ++i;
    }
  }
  cloud_->width = i;
  cloud_->points.resize(i);
}

bool Grid3d::saveGrid(const std::string& grid_path)
{
  auto pf = fopen(grid_path.c_str(), "wb");
  if (!pf)
  {
    ROS_ERROR("[%s] Error opening file %s for writing", ros::this_node::getName().data(), grid_path.c_str());
    return false;
  }

  //! Write grid general info
  fwrite(&grid_size_, sizeof(uint32_t), 1, pf);
  fwrite(&grid_size_x_, sizeof(uint32_t), 1, pf);
  fwrite(&grid_size_y_, sizeof(uint32_t), 1, pf);
  fwrite(&grid_size_z_, sizeof(uint32_t), 1, pf);
  fwrite(&sensor_dev_, sizeof(double), 1, pf);

  //! Write grid cells
  fwrite(grid_.get(), sizeof(GridCell), grid_size_, pf);

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
  fread(&grid_size_, sizeof(uint32_t), 1, pf);
  fread(&grid_size_x_, sizeof(uint32_t), 1, pf);
  fread(&grid_size_y_, sizeof(uint32_t), 1, pf);
  fread(&grid_size_z_, sizeof(uint32_t), 1, pf);
  fread(&sensor_dev, sizeof(double), 1, pf);

  if (std::fabs(sensor_dev - sensor_dev_) >= std::numeric_limits<double>::epsilon())
  {
    ROS_ERROR("[%s] Loaded sensorDev is different", ros::this_node::getName().data());
    return false;
  }

  grid_step_y_ = grid_size_x_;
  grid_step_z_ = grid_size_x_ * grid_size_y_;

  //! Read grid cells
  grid_.reset(new GridCell[grid_size_]);
  fread(grid_.get(), sizeof(GridCell), grid_size_, pf);

  fclose(pf);

  ROS_INFO("[%s] Grid map successfully loaded from %s", ros::this_node::getName().data(), grid_path.c_str());

  return true;
}

void Grid3d::computeGrid()
{
  ROS_INFO("[%s] Computing 3D occupancy grid. This will take some time...", ros::this_node::getName().data());

  //! Alloc the 3D grid
  grid_size_x_ = static_cast<uint32_t>(max_x_ * one_div_res_);
  grid_size_y_ = static_cast<uint32_t>(max_y_ * one_div_res_);
  grid_size_z_ = static_cast<uint32_t>(max_z_ * one_div_res_);
  grid_size_ = grid_size_x_ * grid_size_y_ * grid_size_z_;
  grid_step_y_ = grid_size_x_;
  grid_step_z_ = grid_size_x_ * grid_size_y_;

  grid_.reset(new GridCell[grid_size_]);

  //! Setup kdtree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud_);

  //! Compute the distance to the closest point of the grid
  const float gauss_const1 = static_cast<float>(1. / (sensor_dev_ * sqrt(2 * M_PI)));
  const float gauss_const2 = static_cast<float>(1. / (2. * sensor_dev_ * sensor_dev_));
  uint32_t index;
  float dist;
  pcl::PointXYZ search_point;
  std::vector<int> point_idx_nkn_search(1);
  std::vector<float> point_nkn_squared_distance(1);
  for (uint32_t iz = 0; iz < grid_size_z_; ++iz)
    for (uint32_t iy = 0; iy < grid_size_y_; ++iy)
      for (uint32_t ix = 0; ix < grid_size_x_; ++ix)
      {
        search_point.x = ix * resolution_;
        search_point.y = iy * resolution_;
        search_point.z = iz * resolution_;
        index = ix + iy * grid_step_y_ + iz * grid_step_z_;

        if (kdtree.nearestKSearch(search_point, 1, point_idx_nkn_search, point_nkn_squared_distance) > 0)
        {
          dist = point_nkn_squared_distance[0];
          grid_[index].dist = dist;
          grid_[index].prob = gauss_const1 * expf(-dist * dist * gauss_const2);
        }
        else
        {
          grid_[index].dist = -1.0;
          grid_[index].prob = 0.0;
        }
      }

  ROS_INFO("[%s] Computing 3D occupancy grid done!", ros::this_node::getName().data());
}

inline uint32_t Grid3d::point2grid(const float x, const float y, const float z) const
{
  return static_cast<uint32_t>(x * one_div_res_) + static_cast<uint32_t>(y * one_div_res_) * grid_step_y_ +
         static_cast<uint32_t>(z * one_div_res_) * grid_step_z_;
}

}  // namespace amcl3d
