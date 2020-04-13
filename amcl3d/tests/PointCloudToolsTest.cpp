/*!
 * @file PointCloudToolsTest.h
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

#include <gmock/gmock.h>

#include <PointCloudTools.h>

using namespace amcl3d;

static const std::string DATA_DIR = std::string(PROJECT_SOURCE_DIR) + std::string("/data");
static const std::string TESTSDATA_DIR = std::string(PROJECT_SOURCE_DIR) + std::string("/tests/data");

class PointCloudToolsTest : public ::testing::Test
{
protected:
  void openOcTreeFromBtFile()
  {
    boost::shared_ptr<octomap::OcTree> octo_tree;

    const std::string file_path = DATA_DIR + "/map/mapfile_complete.bt";

    ASSERT_NO_THROW(octo_tree = openOcTree(file_path));
    ASSERT_NE(octo_tree.get(), nullptr);
    ASSERT_GT(octo_tree->size(), 1);

    double min_x, min_y, min_z;
    octo_tree->getMetricMin(min_x, min_y, min_z);
    ASSERT_DOUBLE_EQ(min_x, -17.35);
    ASSERT_DOUBLE_EQ(min_y, -9.5);
    ASSERT_DOUBLE_EQ(min_z, -1.4);

    double max_x, max_y, max_z;
    octo_tree->getMetricMax(max_x, max_y, max_z);
    ASSERT_DOUBLE_EQ(max_x, 8.75);
    ASSERT_DOUBLE_EQ(max_y, 9.7);
    ASSERT_DOUBLE_EQ(max_z, 6.25);

    double resolution = octo_tree->getResolution();
    ASSERT_DOUBLE_EQ(resolution, 0.05);

    _octo_tree = octo_tree;
  }

  boost::shared_ptr<octomap::OcTree> _octo_tree;
};

TEST_F(PointCloudToolsTest, shouldNotOpenOcTreeFromNonExistentFile)
{
  boost::shared_ptr<octomap::OcTree> octo_tree;

  ASSERT_THROW(octo_tree = openOcTree("unknown_file.bt"), std::runtime_error);
  ASSERT_EQ(octo_tree.get(), nullptr);

  ASSERT_THROW(octo_tree = openOcTree("unknown_file.ot"), std::runtime_error);
  ASSERT_EQ(octo_tree.get(), nullptr);

  ASSERT_THROW(octo_tree = openOcTree("unknown_file.unk"), std::runtime_error);
  ASSERT_EQ(octo_tree.get(), nullptr);
}

TEST_F(PointCloudToolsTest, shouldNotOpenOcTreeFromWrongBtFile)
{
  boost::shared_ptr<octomap::OcTree> octo_tree;

  const std::string file_path = TESTSDATA_DIR + "/mapfile_wrong.bt";

  ASSERT_THROW(octo_tree = openOcTree(file_path), std::runtime_error);
  ASSERT_EQ(octo_tree.get(), nullptr);
}

TEST_F(PointCloudToolsTest, shouldNotOpenOcTreeFromWrongOtFile)
{
  boost::shared_ptr<octomap::OcTree> octo_tree;

  const std::string file_path = TESTSDATA_DIR + "/mapfile_wrong.ot";

  ASSERT_THROW(octo_tree = openOcTree(file_path), std::runtime_error);
  ASSERT_EQ(octo_tree.get(), nullptr);
}

TEST_F(PointCloudToolsTest, shouldNotOpenOcTreeFromUnknownExtension)
{
  boost::shared_ptr<octomap::OcTree> octo_tree;

  const std::string file_path = TESTSDATA_DIR + "/mapfile_unknown_extension.unk";

  ASSERT_THROW(octo_tree = openOcTree(file_path), std::runtime_error);
  ASSERT_EQ(octo_tree.get(), nullptr);
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
  ASSERT_NE(octo_tree.get(), nullptr);
  ASSERT_GT(octo_tree->size(), 1);

  double min_x, min_y, min_z;
  octo_tree->getMetricMin(min_x, min_y, min_z);
  ASSERT_DOUBLE_EQ(min_x, -17.35);
  ASSERT_DOUBLE_EQ(min_y, -9.5);
  ASSERT_DOUBLE_EQ(min_z, -1.45);

  double max_x, max_y, max_z;
  octo_tree->getMetricMax(max_x, max_y, max_z);
  ASSERT_DOUBLE_EQ(max_x, 8.8);
  ASSERT_DOUBLE_EQ(max_y, 9.7);
  ASSERT_DOUBLE_EQ(max_z, 6.25);

  double resolution = octo_tree->getResolution();
  ASSERT_DOUBLE_EQ(resolution, 0.05);
}

TEST_F(PointCloudToolsTest, shouldNotComputePointCloudWithNullOctoTree)
{
  PointCloudInfo::Ptr pc_info;

  ASSERT_THROW(pc_info = computePointCloud(nullptr), std::runtime_error);

  ASSERT_EQ(pc_info, nullptr);
}

TEST_F(PointCloudToolsTest, shouldNotComputePointCloudWithEmptyOctoTree)
{
  PointCloudInfo::Ptr pc_info;

  boost::shared_ptr<octomap::OcTree> octo_tree(new octomap::OcTree(0.1));

  ASSERT_THROW(pc_info = computePointCloud(octo_tree), std::runtime_error);

  ASSERT_EQ(pc_info, nullptr);
}

TEST_F(PointCloudToolsTest, shouldComputePointCloud)
{
  PointCloudInfo::Ptr pc_info;

  openOcTreeFromBtFile();

  ASSERT_NO_THROW(pc_info = computePointCloud(_octo_tree));

  ASSERT_NE(pc_info, nullptr);
  ASSERT_NE(pc_info->cloud, nullptr);
  ASSERT_EQ(pc_info->cloud->size(), 179551);
  ASSERT_DOUBLE_EQ(pc_info->octo_min_x, -17.35);
  ASSERT_DOUBLE_EQ(pc_info->octo_min_y, -9.5);
  ASSERT_DOUBLE_EQ(pc_info->octo_min_z, -1.4);
  ASSERT_DOUBLE_EQ(pc_info->octo_max_x, 8.75);
  ASSERT_DOUBLE_EQ(pc_info->octo_max_y, 9.7);
  ASSERT_DOUBLE_EQ(pc_info->octo_max_z, 6.25);
  ASSERT_DOUBLE_EQ(pc_info->octo_resol, 0.05);
}
