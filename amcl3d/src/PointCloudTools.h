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

#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace amcl3d
{

class Grid3dCell
{
public:
  float dist { -1 },
        prob { 0 };
};

class Grid3dInfo
{
public:
  std::unique_ptr<Grid3dCell[]> grid;
  double   max_x  { 0 },
           max_y  { 0 },
           max_z  { 0 };
  uint32_t size   { 0 },
           size_x { 0 },
           size_y { 0 },
           size_z { 0 };
  uint32_t step_y { 0 },
           step_z { 0 };
};

class PointCloudInfo
{
public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  double octo_min_x { 0 },
         octo_min_y { 0 },
         octo_min_z { 0 };
  double octo_max_x { 0 },
         octo_max_y { 0 },
         octo_max_z { 0 };
  double octo_resol { 0 };
};

std::shared_ptr<octomap::OcTree> openOcTree(const std::string& file_path);

//! Load the octomap in PCL for easy nearest neighborhood computation
//! The point-cloud is shifted to have (0,0,0) as min values
PointCloudInfo computePointCloud(std::shared_ptr<octomap::OcTree> octo_tree);

Grid3dInfo computeGrid(const PointCloudInfo &pc_info, const double sensor_dev);

}  // namespace amcl3d
