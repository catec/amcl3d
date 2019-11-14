/*!
 * @file PointCloudToolsTest.cpp
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

#include <PointCloudTools.h>

#include "common_tests.h"

static const std::string TEST_DIR = std::string("/tmp/JsonMissionFileTest");

using namespace amcl3d;

class PointCloudToolsTest : public ::testing::Test
{
protected:
  void openOcTreeFromBtFile()
  {
    boost::shared_ptr<octomap::OcTree> octo_tree;

    const std::string file_path = DATA_DIR + "/map/mapfile_complete.bt";

    ASSERT_NO_THROW(octo_tree = openOcTree(file_path));
    ASSERT_NE(nullptr, octo_tree.get());
    ASSERT_GT(octo_tree->size(), 1);

    double min_x, min_y, min_z;
    octo_tree->getMetricMin(min_x, min_y, min_z);
    ASSERT_DOUBLE_EQ(-17.35, min_x);
    ASSERT_DOUBLE_EQ(-9.5, min_y);
    ASSERT_DOUBLE_EQ(-1.4, min_z);

    double max_x, max_y, max_z;
    octo_tree->getMetricMax(max_x, max_y, max_z);
    ASSERT_DOUBLE_EQ(8.75, max_x);
    ASSERT_DOUBLE_EQ(9.7, max_y);
    ASSERT_DOUBLE_EQ(6.25, max_z);

    double resolution = octo_tree->getResolution();
    ASSERT_DOUBLE_EQ(0.05, resolution);

    _octo_tree = octo_tree;
  }

  virtual void SetUp()
  {
    boost::filesystem::create_directory(TEST_DIR);
  }

  virtual void TearDown()
  {
     boost::filesystem::remove_all(TEST_DIR);
  }

  boost::shared_ptr<octomap::OcTree> _octo_tree;
};

/** openOcTree method tests **/

TEST_F(PointCloudToolsTest, shouldNotOpenOcTreeFromNonExistentFile)
{
  boost::shared_ptr<octomap::OcTree> octo_tree;

  const std::string file_path_list[] =
    { "unknown_file.bt", "unknown_file.ot", "unknown_file.unk" };
  for (int i = 0; i < 3; ++i)
  {
    const std::string expected_what = std::string("Cannot find file ") + file_path_list[i];

    ASSERT_THROW_WHAT(octo_tree = openOcTree(file_path_list[i]),
                      std::runtime_error, expected_what);
    ASSERT_EQ(nullptr, octo_tree.get());
  }
}

TEST_F(PointCloudToolsTest, shouldNotOpenOcTreeFromWrongBtFile)
{
  boost::shared_ptr<octomap::OcTree> octo_tree;

  const std::string file_path = TESTSDATA_DIR + "/mapfile_wrong.bt";
  const std::string expected_what("OcTree cannot be read");

  ASSERT_THROW_WHAT(octo_tree = openOcTree(file_path),
                    std::runtime_error, expected_what);
  ASSERT_EQ(nullptr, octo_tree.get());
}

TEST_F(PointCloudToolsTest, shouldNotOpenOcTreeFromWrongOtFile)
{
  boost::shared_ptr<octomap::OcTree> octo_tree;

  const std::string file_path = TESTSDATA_DIR + "/mapfile_wrong.ot";
  const std::string expected_what = std::string("OcTree cannot be created from file ") + file_path;

  ASSERT_THROW_WHAT(octo_tree = openOcTree(file_path),
                    std::runtime_error, expected_what);
  ASSERT_EQ(nullptr, octo_tree.get());
}

TEST_F(PointCloudToolsTest, shouldNotOpenOcTreeFromUnknownExtension)
{
  boost::shared_ptr<octomap::OcTree> octo_tree;

  const std::string file_path = TESTSDATA_DIR + "/mapfile_unknown_extension.unk";
  const std::string expected_what = std::string("OcTree cannot be created from file ") + file_path;

  ASSERT_THROW_WHAT(octo_tree = openOcTree(file_path),
                    std::runtime_error, expected_what);
  ASSERT_EQ(nullptr, octo_tree.get());
}

TEST_F(PointCloudToolsTest, shouldOpenOcTreeFromBtFile)
{
  openOcTreeFromBtFile();
}

TEST_F(PointCloudToolsTest, shouldOpenOcTreeFromOtFile)
{
  boost::shared_ptr<octomap::OcTree> octo_tree;

  const std::string file_path = DATA_DIR + "/map/mapfile_complete_ot.ot";

  ASSERT_NO_THROW(octo_tree = openOcTree(file_path));
  ASSERT_NE(nullptr, octo_tree.get());
  ASSERT_GT(octo_tree->size(), 1);

  double min_x, min_y, min_z;
  octo_tree->getMetricMin(min_x, min_y, min_z);
  ASSERT_DOUBLE_EQ(-17.35, min_x);
  ASSERT_DOUBLE_EQ(-9.5, min_y);
  ASSERT_DOUBLE_EQ(-1.45, min_z);

  double max_x, max_y, max_z;
  octo_tree->getMetricMax(max_x, max_y, max_z);
  ASSERT_DOUBLE_EQ(8.8, max_x);
  ASSERT_DOUBLE_EQ(9.7, max_y);
  ASSERT_DOUBLE_EQ(6.25, max_z);

  double resolution = octo_tree->getResolution();
  ASSERT_DOUBLE_EQ(0.05, resolution);
}

/** computePointCloud method tests **/

TEST_F(PointCloudToolsTest, shouldNotComputePointCloudWithNullOctoTree)
{
  PointCloudInfo::Ptr pc_info;

  const std::string expected_what("OcTree is NULL");

  ASSERT_THROW_WHAT(pc_info = computePointCloud(nullptr),
                    std::runtime_error, expected_what);
  ASSERT_EQ(nullptr, pc_info);
}

TEST_F(PointCloudToolsTest, shouldNotComputePointCloudWithEmptyOctoTree)
{
  PointCloudInfo::Ptr pc_info;

  boost::shared_ptr<octomap::OcTree> octo_tree(new octomap::OcTree(0.1));

  const std::string expected_what("OcTree is empty");

  ASSERT_THROW_WHAT(pc_info = computePointCloud(octo_tree),
                    std::runtime_error, expected_what);
  ASSERT_EQ(nullptr, pc_info);
}

TEST_F(PointCloudToolsTest, shouldComputePointCloud)
{
  PointCloudInfo::Ptr pc_info;

  openOcTreeFromBtFile();

  ASSERT_NO_THROW(pc_info = computePointCloud(_octo_tree));

  ASSERT_NE(nullptr, pc_info);
  ASSERT_NE(nullptr, pc_info->cloud);
  ASSERT_EQ(179551, pc_info->cloud->size());
  ASSERT_DOUBLE_EQ(-17.35, pc_info->octo_min_x);
  ASSERT_DOUBLE_EQ(-9.5, pc_info->octo_min_y);
  ASSERT_DOUBLE_EQ(-1.4, pc_info->octo_min_z);
  ASSERT_DOUBLE_EQ(8.75, pc_info->octo_max_x);
  ASSERT_DOUBLE_EQ(9.7, pc_info->octo_max_y);
  ASSERT_DOUBLE_EQ(6.25, pc_info->octo_max_z);
  ASSERT_DOUBLE_EQ(0.05, pc_info->octo_resol);
}

/** computeGrid method tests **/

TEST_F(PointCloudToolsTest, shouldNotComputeGridWithSensorDevLessOrEqualThanZero)
{
  Grid3dInfo::Ptr grid_info;

  const std::string expected_what("SensorDev is not greater than zero");

  for (int value = -10; value <= 0; ++value)
  {
    ASSERT_THROW_WHAT(grid_info = computeGrid(nullptr, value),
                      std::runtime_error, expected_what);
    ASSERT_EQ(nullptr, grid_info);
  }

  ASSERT_THROW_NO_WHAT(grid_info = computeGrid(nullptr, 1),
                       std::runtime_error, expected_what);
  ASSERT_EQ(nullptr, grid_info);
}

TEST_F(PointCloudToolsTest, shouldNotComputeGridWithNullPointCloudInfo)
{
  Grid3dInfo::Ptr grid_info;

  const std::string expected_what("PointCloudInfo is NULL");

  ASSERT_THROW_WHAT(grid_info = computeGrid(nullptr, DEFAULT_SENSOR_DEV),
                    std::runtime_error, expected_what);
  ASSERT_EQ(nullptr, grid_info);
}

TEST_F(PointCloudToolsTest, shouldNotComputeGridWithResolutionLessOrEqualThanZero)
{
  Grid3dInfo::Ptr grid_info;

  PointCloudInfo::Ptr pc_info(new PointCloudInfo());

  const std::string expected_what("Octo resolution is not greater than zero");

  for (int value = -10; value <= 0; ++value)
  {
    pc_info->octo_resol = value;

    ASSERT_THROW_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                      std::runtime_error, expected_what);
    ASSERT_EQ(nullptr, grid_info);
  }

  pc_info->octo_resol = 1;

  ASSERT_THROW_NO_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                       std::runtime_error, expected_what);
  ASSERT_EQ(nullptr, grid_info);
}

TEST_F(PointCloudToolsTest, shouldNotComputeGridWithMinXPositionGreaterThanMaxXPosition)
{
  Grid3dInfo::Ptr grid_info;

  PointCloudInfo::Ptr pc_info(new PointCloudInfo());
  pc_info->octo_resol = 1;

  const std::string expected_what("Octo minimum X position is greater or equal than maximum X position");

  for (int value = -10; value <= 10; ++value)
  {
    pc_info->octo_max_x = value;
    pc_info->octo_min_x = pc_info->octo_max_x + 1;

    ASSERT_THROW_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                      std::runtime_error, expected_what);
    ASSERT_EQ(nullptr, grid_info);

    pc_info->octo_max_x = value;
    pc_info->octo_min_x = pc_info->octo_max_x;

    ASSERT_THROW_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                      std::runtime_error, expected_what);
    ASSERT_EQ(nullptr, grid_info);

    pc_info->octo_max_x = value;
    pc_info->octo_min_x = pc_info->octo_max_x - 1;

    ASSERT_THROW_NO_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                         std::runtime_error, expected_what);
    ASSERT_EQ(nullptr, grid_info);
  }
}

TEST_F(PointCloudToolsTest, shouldNotComputeGridWithMinYPositionGreaterThanMaxYPosition)
{
  Grid3dInfo::Ptr grid_info;

  PointCloudInfo::Ptr pc_info(new PointCloudInfo());
  pc_info->octo_resol = 1;
  pc_info->octo_max_x = 2;
  pc_info->octo_min_x = pc_info->octo_max_x - 1;

  const std::string expected_what("Octo minimum Y position is greater or equal than maximum Y position");

  for (int value = -10; value <= 10; ++value)
  {
    pc_info->octo_max_y = value;
    pc_info->octo_min_y = pc_info->octo_max_y + 1;

    ASSERT_THROW_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                      std::runtime_error, expected_what);
    ASSERT_EQ(nullptr, grid_info);

    pc_info->octo_max_y = value;
    pc_info->octo_min_y = pc_info->octo_max_y;

    ASSERT_THROW_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                      std::runtime_error, expected_what);
    ASSERT_EQ(nullptr, grid_info);

    pc_info->octo_max_y = value;
    pc_info->octo_min_y = pc_info->octo_max_y - 1;

    ASSERT_THROW_NO_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                         std::runtime_error, expected_what);
    ASSERT_EQ(nullptr, grid_info);
  }
}

TEST_F(PointCloudToolsTest, shouldNotComputeGridWithMinZPositionGreaterThanMaxZPosition)
{
  Grid3dInfo::Ptr grid_info;

  PointCloudInfo::Ptr pc_info(new PointCloudInfo());
  pc_info->octo_resol = 1;
  pc_info->octo_max_x = 2;
  pc_info->octo_min_x = pc_info->octo_max_x - 1;
  pc_info->octo_max_y = 3;
  pc_info->octo_min_y = pc_info->octo_max_y - 1;

  const std::string expected_what("Octo minimum Z position is greater or equal than maximum Z position");

  for (int value = -10; value <= 10; ++value)
  {
    pc_info->octo_max_z = value;
    pc_info->octo_min_z = pc_info->octo_max_z + 1;

    ASSERT_THROW_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                      std::runtime_error, expected_what);
    ASSERT_EQ(nullptr, grid_info);

    pc_info->octo_max_z = value;
    pc_info->octo_min_z = pc_info->octo_max_z;

    ASSERT_THROW_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                      std::runtime_error, expected_what);
    ASSERT_EQ(nullptr, grid_info);

    pc_info->octo_max_z = value;
    pc_info->octo_min_z = pc_info->octo_max_z - 1;

    ASSERT_THROW_NO_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                         std::runtime_error, expected_what);
    ASSERT_EQ(nullptr, grid_info);
  }
}

TEST_F(PointCloudToolsTest, shouldNotComputeGridWithNullPclPointCloud)
{
  Grid3dInfo::Ptr grid_info;

  PointCloudInfo::Ptr pc_info(new PointCloudInfo());
  pc_info->octo_resol = 1;
  pc_info->octo_max_x = 2;
  pc_info->octo_min_x = pc_info->octo_max_x - 1;
  pc_info->octo_max_y = 3;
  pc_info->octo_min_y = pc_info->octo_max_y - 1;
  pc_info->octo_max_z = 4;
  pc_info->octo_min_z = pc_info->octo_max_z - 1;

  const std::string expected_what("PCL::PointCloud is NULL");

  ASSERT_THROW_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                    std::runtime_error, expected_what);
  ASSERT_EQ(nullptr, grid_info);
}

TEST_F(PointCloudToolsTest, shouldNotComputeGridWithEmptyPclPointCloud)
{
  Grid3dInfo::Ptr grid_info;

  PointCloudInfo::Ptr pc_info(new PointCloudInfo());
  pc_info->octo_resol = 1;
  pc_info->octo_max_x = 2;
  pc_info->octo_min_x = pc_info->octo_max_x - 1;
  pc_info->octo_max_y = 3;
  pc_info->octo_min_y = pc_info->octo_max_y - 1;
  pc_info->octo_max_z = 4;
  pc_info->octo_min_z = pc_info->octo_max_z - 1;
  pc_info->cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

  const std::string expected_what("PCL::PointCloud is empty");

  ASSERT_THROW_WHAT(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV),
                    std::runtime_error, expected_what);
  ASSERT_EQ(nullptr, grid_info);
}

TEST_F(PointCloudToolsTest, shouldComputeGrid)
{
  Grid3dInfo::Ptr grid_info;

  const double bounds = 10;

  const double res_list[] = { 0.1, 0.5, 1, 5, 10 };
  for (int i = 0; i < 5; ++i)
  {
    PointCloudInfo::Ptr pc_info(new PointCloudInfo());
    pc_info->octo_resol = res_list[i];
    pc_info->octo_max_x = bounds;
    pc_info->octo_min_x = -bounds;
    pc_info->octo_max_y = bounds;
    pc_info->octo_min_y = -bounds;
    pc_info->octo_max_z = bounds;
    pc_info->octo_min_z = -bounds;
    pc_info->cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ point;
    for (double iz = pc_info->octo_min_z; iz < pc_info->octo_max_z; iz+=pc_info->octo_resol)
    {
      for (double iy = pc_info->octo_min_y; iy < pc_info->octo_max_y; iy+=pc_info->octo_resol)
      {
        for (double ix = pc_info->octo_min_x; ix < pc_info->octo_max_x; ix+=pc_info->octo_resol)
        {
          point.x = ix;
          point.y = iy;
          point.z = iz;
          pc_info->cloud->push_back(point);
        }
      }
    }

    ASSERT_NO_THROW(grid_info = computeGrid(pc_info, DEFAULT_SENSOR_DEV));
    ASSERT_NE(nullptr, grid_info);
    ASSERT_DOUBLE_EQ(DEFAULT_SENSOR_DEV, grid_info->sensor_dev);
    const auto expected_size = static_cast<uint32_t>(2*bounds / pc_info->octo_resol);
    ASSERT_EQ(expected_size, grid_info->size_x);
    ASSERT_EQ(expected_size, grid_info->size_y);
    ASSERT_EQ(expected_size, grid_info->size_z);
    ASSERT_EQ(expected_size, grid_info->step_y);
    ASSERT_EQ(expected_size * expected_size, grid_info->step_z);
    ASSERT_EQ(expected_size * expected_size * expected_size, grid_info->grid.size());
    const auto expected_prob = static_cast<float>(1. / (grid_info->sensor_dev * sqrt(2 * M_PI)));
    for (uint32_t index = 0; index < grid_info->grid.size(); ++index)
    {
      ASSERT_NEAR(0.f, grid_info->grid[index].dist, 1e-20f);
      ASSERT_FLOAT_EQ(expected_prob, grid_info->grid[index].prob);
    }
  }
}

/** saveGrid method tests **/

TEST_F(PointCloudToolsTest, shouldNotSaveGridWithNullGrid3dInfo)
{
  Grid3dInfo::Ptr grid_info;

  const std::string file_path = TEST_DIR + "/test_file.grid";

  const std::string expected_what("Grid3dInfo is NULL");

  ASSERT_THROW_WHAT(saveGrid(grid_info, file_path),
                    std::runtime_error, expected_what);
}

TEST_F(PointCloudToolsTest, shouldNotSaveGridWithEmptyGrid3dInfo)
{
  Grid3dInfo::Ptr grid_info(new Grid3dInfo());

  const std::string file_path = TEST_DIR + "/test_file.grid";

  const std::string expected_what("Grid3dInfo is empty");

  ASSERT_THROW_WHAT(saveGrid(grid_info, file_path),
                    std::runtime_error, expected_what);
}

TEST_F(PointCloudToolsTest, shouldNotSaveGridWithInvalidGrid3d)
{
  Grid3dInfo::Ptr grid_info(new Grid3dInfo());
  grid_info->grid.resize(1);

  const std::string file_path = TEST_DIR + "/test_file.grid";

  const std::string expected_what("Grid3dInfo is invalid");

  ASSERT_THROW_WHAT(saveGrid(grid_info, file_path),
                    std::runtime_error, expected_what);
}

TEST_F(PointCloudToolsTest, shouldNotSaveGridWithEmptyFilePath)
{
  Grid3dInfo::Ptr grid_info(new Grid3dInfo());
  grid_info->size_x = 1;
  grid_info->size_y = 1;
  grid_info->size_z = 1;
  grid_info->grid.resize(1);

  const std::string file_path = "";

  const std::string expected_what = std::string("Cannot be created file ") + file_path;

  ASSERT_THROW_WHAT(saveGrid(grid_info, file_path),
                    std::runtime_error, expected_what);
}

TEST_F(PointCloudToolsTest, shouldNotSaveGridWithNonExistentFilePath)
{
  Grid3dInfo::Ptr grid_info(new Grid3dInfo());
  grid_info->size_x = 1;
  grid_info->size_y = 1;
  grid_info->size_z = 1;
  grid_info->grid.resize(1);

  const std::string file_path("/tmp/UnknownFolder/test_file.grid");

  const std::string expected_what = std::string("Cannot be created file ") + file_path;

  ASSERT_THROW_WHAT(saveGrid(grid_info, file_path),
                    std::runtime_error, expected_what);
}

TEST_F(PointCloudToolsTest, shouldSaveGrid)
{
  Grid3dInfo::Ptr grid_info(new Grid3dInfo());
  grid_info->sensor_dev = DEFAULT_SENSOR_DEV;
  grid_info->size_x = 10;
  grid_info->size_y = 10;
  grid_info->size_z = 10;
  grid_info->grid.resize(1000);

  const std::string file_path = TEST_DIR + "/test_file.grid";

  ASSERT_NO_THROW(saveGrid(grid_info, file_path));
}
