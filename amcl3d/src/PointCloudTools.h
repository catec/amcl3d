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

#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace amcl3d
{
/*! \brief Class that contains the size of the cell grid.
 */
class Grid3dCell
{
public:
  float dist{ -1 }; /*!< Distance */
  float prob{ 0 };  /*!< Probability */
};

/*! \brief Class that contains the information of the grid.
 */
class Grid3dInfo
{
public:
  typedef boost::shared_ptr<Grid3dInfo> Ptr;            /*!< Grid information shared point */
  typedef boost::shared_ptr<const Grid3dInfo> ConstPtr; /*!< Grid information shared point (const)*/

  std::vector<Grid3dCell> grid; /*!< Vector of cells */
  double sensor_dev{ 0 };       /*!< Desviation of the 3d point cloud sensor */
  uint32_t size_x{ 0 };         /*!< Grid size of x-axis */
  uint32_t size_y{ 0 };         /*!< Grid size of y-axis */
  uint32_t size_z{ 0 };         /*!< Grid size of z-axis */
  uint32_t step_y{ 0 };         /*!< Step to convert from matrix to vector in y-axis */
  uint32_t step_z{ 0 };         /*!< Step to convert from matrix to vector in z-axis */
};

/*! \brief Class that contains the information of the point cloud.
 */
class PointCloudInfo
{
public:
  typedef boost::shared_ptr<PointCloudInfo> Ptr;            /*!< Point cloud information shared point */
  typedef boost::shared_ptr<const PointCloudInfo> ConstPtr; /*!< Point cloud information shared point (const)*/

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; /*!< Pcl point cloud */
  double octo_min_x{ 0 };                    /*!< Minimum x-axis value of octomap */
  double octo_min_y{ 0 };                    /*!< Minimum y-axis value of octomap */
  double octo_min_z{ 0 };                    /*!< Minimum z-axis value of octomap */
  double octo_max_x{ 0 };                    /*!< Maximum x-axis value of octomap */
  double octo_max_y{ 0 };                    /*!< Maximum y-axis value of octomap */
  double octo_max_z{ 0 };                    /*!< Maximum z-axis value of octomap */
  double octo_resol{ 0 };                    /*!< Octomap Resolution */
};

/*! \brief To open the octomap.
 *
 * \param file_path Route of the octomap environment.
 * \return OcTree Octomap information shared point
 *
 * It is responsible for opening the octomapa using the route that the user provides. It check the route and the
 * correctly creation of the octree.
 */
boost::shared_ptr<octomap::OcTree> openOcTree(const std::string& file_path);

/*! \brief Load the octomap in PCL for easy nearest neighborhood computation.
 *
 * \param octo_tree Shared point that contains the information of the environment map.
 * \return PointCloudInfo Point cloud information associated with the map.
 *
 * It is responsible for opening the octomapa using the route that the user provides. It check the route and the
 * correctly creation of the octree. The point-cloud is shifted to have (0,0,0) as min values
 */
PointCloudInfo::Ptr computePointCloud(boost::shared_ptr<octomap::OcTree> octo_tree);

/*! \brief Load the octomap in PCL for easy nearest neighborhood computation.
 *
 * \param pc_info Point cloud information associated with the map.
 * \param sensor_dev Desviation of the 3d point cloud sensor.
 * \return Grid3dInfo Grid information associated with the map.
 *
 * It verifies the information in the point cloud to check if it is not null. Use the sensor deviation to calculate and
 * save the grid size as well as the distance information between the cloud points to the nearest point on the grid.
 */
Grid3dInfo::Ptr computeGrid(PointCloudInfo::Ptr pc_info, const double sensor_dev);

}  // namespace amcl3d
