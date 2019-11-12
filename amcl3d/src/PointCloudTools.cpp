/*!
 * @file PointCloudTools.cpp
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

#include <boost/filesystem.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include "PointCloudTools.h"

namespace amcl3d
{
boost::shared_ptr<octomap::OcTree> openOcTree(const std::string& file_path)
{
  if (!boost::filesystem::exists(file_path))
    throw std::runtime_error(std::string("Cannot find file ") + file_path);

  boost::shared_ptr<octomap::OcTree> octo_tree;

  if (file_path.compare(file_path.length() - 3, 3, ".bt") == 0)
  {
    octo_tree.reset(new octomap::OcTree(0.1));

    if (!octo_tree->readBinary(file_path))
      throw std::runtime_error("OcTree cannot be read");
  }
  else if (file_path.compare(file_path.length() - 3, 3, ".ot") == 0)
  {
    octo_tree.reset(dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::read(file_path)));
  }

  if (!octo_tree)
    throw std::runtime_error(std::string("OcTree cannot be created from file ") + file_path);

  return octo_tree;
}

PointCloudInfo::Ptr computePointCloud(boost::shared_ptr<octomap::OcTree> octo_tree)
{
  if (!octo_tree)
    throw std::runtime_error("OcTree is NULL");

  const uint32_t octo_size = octo_tree->size();
  if (octo_size <= 1)
    throw std::runtime_error("OcTree is empty");

  PointCloudInfo::Ptr pc_info(new PointCloudInfo());

  octo_tree->getMetricMin(pc_info->octo_min_x, pc_info->octo_min_y, pc_info->octo_min_z);
  octo_tree->getMetricMax(pc_info->octo_max_x, pc_info->octo_max_y, pc_info->octo_max_z);
  pc_info->octo_resol = octo_tree->getResolution();

  pc_info->cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PointXYZ point;
  for (octomap::OcTree::leaf_iterator it = octo_tree->begin_leafs(); it != octo_tree->end_leafs(); ++it)
  {
    if (it != nullptr && octo_tree->isNodeOccupied(*it))
    {
      point.x = static_cast<float>(it.getX());
      point.y = static_cast<float>(it.getY());
      point.z = static_cast<float>(it.getZ());

      pc_info->cloud->push_back(point);
    }
  }

  return pc_info;
}

Grid3dInfo::Ptr computeGrid(PointCloudInfo::Ptr pc_info, const double sensor_dev)
{
  if (!pc_info)
    throw std::runtime_error("PointCloudInfo is NULL");

  Grid3dInfo::Ptr grid_info(new Grid3dInfo());
  grid_info->sensor_dev = sensor_dev;

  //! Alloc the 3D grid
  const auto octo_size_x = pc_info->octo_max_x - pc_info->octo_min_x;
  const auto octo_size_y = pc_info->octo_max_y - pc_info->octo_min_y;
  const auto octo_size_z = pc_info->octo_max_z - pc_info->octo_min_z;
  grid_info->size_x = static_cast<uint32_t>(octo_size_x / pc_info->octo_resol);
  grid_info->size_y = static_cast<uint32_t>(octo_size_y / pc_info->octo_resol);
  grid_info->size_z = static_cast<uint32_t>(octo_size_z / pc_info->octo_resol);

  grid_info->step_y = grid_info->size_x;
  grid_info->step_z = grid_info->size_x * grid_info->size_y;

  const auto grid_size = grid_info->size_x * grid_info->size_y * grid_info->size_z;
  grid_info->grid.resize(grid_size);

  //! Setup kdtree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(pc_info->cloud);

  //! Compute the distance to the closest point of the grid
  const float gauss_const1 = static_cast<float>(1. / (grid_info->sensor_dev * sqrt(2 * M_PI)));
  const float gauss_const2 = static_cast<float>(1. / (2. * grid_info->sensor_dev * grid_info->sensor_dev));
  uint32_t index;
  float dist;
  pcl::PointXYZ search_point;
  std::vector<int> point_idx_nkn_search(1);
  std::vector<float> point_nkn_squared_distance(1);
  for (uint32_t iz = 0; iz < grid_info->size_z; ++iz)
  {
    for (uint32_t iy = 0; iy < grid_info->size_y; ++iy)
    {
      for (uint32_t ix = 0; ix < grid_info->size_x; ++ix)
      {
        search_point.x = pc_info->octo_min_x + (ix * pc_info->octo_resol);
        search_point.y = pc_info->octo_min_y + (iy * pc_info->octo_resol);
        search_point.z = pc_info->octo_min_z + (iz * pc_info->octo_resol);

        index = ix + iy * grid_info->step_y + iz * grid_info->step_z;

        if (kdtree.nearestKSearch(search_point, 1, point_idx_nkn_search, point_nkn_squared_distance) > 0)
        {
          dist = point_nkn_squared_distance[0];
          grid_info->grid[index].dist = dist;
          grid_info->grid[index].prob = gauss_const1 * expf(-dist * dist * gauss_const2);
        }
        else
        {
          grid_info->grid[index].dist = -1.0;
          grid_info->grid[index].prob = 0.0;
        }
      }
    }
  }

  return grid_info;
}

}  // namespace amcl3d
