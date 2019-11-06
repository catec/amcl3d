/*!
 * @file PointCloudTools.h
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

#include <memory>

#include <boost/filesystem.hpp>

#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace amcl3d
{

std::shared_ptr<octomap::OcTree> openOcTree(const std::string& file_path)
{
  if (!boost::filesystem::exists(file_path))
    throw std::runtime_error(std::string("Cannot find file ") + file_path);

  std::shared_ptr<octomap::OcTree> octo_tree;

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

//! Load the octomap in PCL for easy nearest neighborhood computation
//! The point-cloud is shifted to have (0,0,0) as min values
pcl::PointCloud<pcl::PointXYZ>::Ptr computePointCloud(std::shared_ptr<octomap::OcTree> octo_tree)
{
  if (!octo_tree)
    throw std::runtime_error("OcTree is NULL");

  const uint32_t octo_size = octo_tree->size();
  if (octo_size <= 1)
    throw std::runtime_error("OcTree is empty");

  double octo_min_x, octo_min_y, octo_min_z;
  octo_tree->getMetricMin(octo_min_x, octo_min_y, octo_min_z);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width  = octo_size;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  uint32_t i = 0;
  for (octomap::OcTree::leaf_iterator it = octo_tree->begin_leafs(); it != octo_tree->end_leafs(); ++it)
  {
    if (it != nullptr && octo_tree->isNodeOccupied(*it))
    {
      cloud->points[i].x = static_cast<float>(it.getX()) - octo_min_x;
      cloud->points[i].y = static_cast<float>(it.getY()) - octo_min_y;
      cloud->points[i].z = static_cast<float>(it.getZ()) - octo_min_z;
      ++i;
    }
  }
  cloud->width = i;
  cloud->points.resize(i);

  return cloud;
}

}  // namespace amcl3d
