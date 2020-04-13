#include <gmock/gmock.h>

#include <Grid3d.h>

using namespace amcl3d;

static const double DEFAULT_SENSOR_DEV = 0.05;

class Grid3dTest : public ::testing::Test
{
protected:
  Grid3d _sut;
};

TEST_F(Grid3dTest, openTest)
{
  std::string source = PROJECT_SOURCE_DIR;

  // Open .bt
  std::string map_path_bt = source + "/data/map/mapfile_complete.bt";
  bool open_true_bt = _sut.open(map_path_bt, DEFAULT_SENSOR_DEV);
  EXPECT_EQ(open_true_bt, true);

  // Open .ot
  std::string map_path_ot = source + "/data/map/mapfile_complete_ot.ot";
  bool open_true_ot = _sut.open(map_path_ot, DEFAULT_SENSOR_DEV);
  EXPECT_EQ(open_true_ot, true);

  // Open null .bt
  std::string map_path_nullbt = source + "/tests/data/mapfile_null.bt";
  bool open_false_nullbt = _sut.open(map_path_nullbt, DEFAULT_SENSOR_DEV);
  EXPECT_EQ(open_false_nullbt, false);

  // Open null .ot
  std::string map_path_nullot = source + "/tests/data/mapfile_null.ot";
  bool open_false_nullot = _sut.open(map_path_nullot, DEFAULT_SENSOR_DEV);
  EXPECT_EQ(open_false_nullot, false);

  // Open map_path.length() <= 3
  std::string map_path_false = ".bt";
  bool open_false = _sut.open(map_path_false, DEFAULT_SENSOR_DEV);
  EXPECT_EQ(open_false, false);
}

TEST_F(Grid3dTest, buildGridSliceMsgTest)
{
  ros::Time::init();
  std::string source = PROJECT_SOURCE_DIR;
  std::string map_path = source + "/data/map/mapfile_complete.bt";

  nav_msgs::OccupancyGrid msg;
  float z_negative = -100;
  float z_positive = 0;

  // Deserialize msg
  std::ifstream ifs(source + "/tests/data/nav_msg.bin", std::ios::in | std::ios::binary);
  ifs.seekg(0, std::ios::end);
  std::streampos end = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  std::streampos begin = ifs.tellg();

  uint32_t file_size = end - begin;
  boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
  ifs.read((char*)ibuffer.get(), file_size);
  ros::serialization::IStream istream(ibuffer.get(), file_size);
  ros::serialization::deserialize(istream, msg);
  ifs.close();

  // Without Open
  bool result_open = _sut.buildGridSliceMsg(z_negative, msg);
  EXPECT_EQ(result_open, false);

  // Open
  bool open_true = _sut.open(map_path, DEFAULT_SENSOR_DEV);
  EXPECT_EQ(open_true, true);

  // Z negative
  bool result_negative = _sut.buildGridSliceMsg(z_negative, msg);
  EXPECT_EQ(result_negative, false);

  // Z positive
  bool result_positive = _sut.buildGridSliceMsg(z_positive, msg);
  EXPECT_EQ(result_positive, true);
}

TEST_F(Grid3dTest, buildMapPointCloudMsgTest)
{
  std::string source = PROJECT_SOURCE_DIR;
  std::string map_path = source + "/data/map/mapfile_complete.bt";

  sensor_msgs::PointCloud2 msg;

  // Deserialize msg
  std::ifstream ifs(source + "/tests/data/mappointcloud_msg.bin", std::ios::in | std::ios::binary);
  ifs.seekg(0, std::ios::end);
  std::streampos end = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  std::streampos begin = ifs.tellg();

  uint32_t file_size = end - begin;
  boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
  ifs.read((char*)ibuffer.get(), file_size);
  ros::serialization::IStream istream(ibuffer.get(), file_size);
  ros::serialization::deserialize(istream, msg);
  ifs.close();

  // cloud_ = false
  bool result_false = _sut.buildMapPointCloudMsg(msg);
  EXPECT_EQ(result_false, false);

  // Open
  bool open_true = _sut.open(map_path, DEFAULT_SENSOR_DEV);
  EXPECT_EQ(open_true, true);

  // cloud_ = true
  bool result_true = _sut.buildMapPointCloudMsg(msg);
  EXPECT_EQ(result_true, true);
}

TEST_F(Grid3dTest, computeCloudWeightTest)
{
  std::string source = PROJECT_SOURCE_DIR;
  std::string map_path = source + "/data/map/mapfile_complete.bt";

  geometry_msgs::PoseArray msg;

  // Expect
  float tx = 20.017967;
  float ty = 10.140815;
  float tz = 3.372801;
  float a = 0.166781;
  float wp_real = 3.8109049797058105;

  // Deserialize msg
  std::ifstream ifs(source + "/tests/data/grid_info.bin", std::ios::in | std::ios::binary);
  ifs.seekg(0, std::ios::end);
  std::streampos end = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  std::streampos begin = ifs.tellg();

  uint32_t file_size = end - begin;
  boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
  ifs.read((char*)ibuffer.get(), file_size);
  ros::serialization::IStream istream(ibuffer.get(), file_size);
  ros::serialization::deserialize(istream, msg);
  ifs.close();

  pcl::PointCloud<pcl::PointXYZ>::Ptr points_grid(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointXYZ point_grid;
  for (int i = 0; i < msg.poses.size(); i++)
  {
    point_grid.x = msg.poses[i].position.x;
    point_grid.y = msg.poses[i].position.y;
    point_grid.z = msg.poses[i].position.z;

    points_grid->push_back(point_grid);
  }

  // computeCloudWeigh without Open
  float result_0 = _sut.computeCloudWeight(points_grid, 20.017967, 10.140815, 3.372801, 0, 0, 0.166781);
  EXPECT_EQ(result_0, 0);

  // Open
  bool open = _sut.open(map_path, DEFAULT_SENSOR_DEV);

  // computeCloudWeight in known point
  float result = _sut.computeCloudWeight(points_grid, 20.017967, 10.140815, 3.372801, 0, 0, 0.166781);
  EXPECT_NEAR(result, wp_real, 0.0001);
}

TEST_F(Grid3dTest, computeCloudWeightParticlesTest)
{
  std::vector<float> wp_test;
  float wtp = 3301.69;

  std::string source = PROJECT_SOURCE_DIR;
  std::string map_path = source + "/data/map/mapfile_complete.bt";

  bool open = _sut.open(map_path, DEFAULT_SENSOR_DEV);

  geometry_msgs::PoseArray msg_pos;
  geometry_msgs::PoseArray msg;
  std::vector<float> tx_vector;
  std::vector<float> ty_vector;
  std::vector<float> tz_vector;
  std::vector<float> ta_vector;
  std::vector<float> wp_vector;

  // Deserialize the points
  std::ifstream ifs(source + "/tests/data/grid_info.bin", std::ios::in | std::ios::binary);
  ifs.seekg(0, std::ios::end);
  std::streampos end = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  std::streampos begin = ifs.tellg();

  uint32_t file_size = end - begin;
  boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
  ifs.read((char*)ibuffer.get(), file_size);
  ros::serialization::IStream istream(ibuffer.get(), file_size);
  ros::serialization::deserialize(istream, msg);
  ifs.close();

  pcl::PointCloud<pcl::PointXYZ>::Ptr points_grid(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointXYZ point_grid;
  for (int i = 0; i < msg.poses.size(); i++)
  {
    point_grid.x = msg.poses[i].position.x;
    point_grid.y = msg.poses[i].position.y;
    point_grid.z = msg.poses[i].position.z;

    points_grid->push_back(point_grid);
  }

  // Deserialize information about particles
  std::ifstream ifs_pos(source + "/tests/data/particle_info.bin", std::ios::in | std::ios::binary);
  ifs_pos.seekg(0, std::ios::end);
  std::streampos end_pos = ifs_pos.tellg();
  ifs_pos.seekg(0, std::ios::beg);
  std::streampos begin_pos = ifs_pos.tellg();

  uint32_t file_size_pos = end_pos - begin_pos;
  boost::shared_array<uint8_t> ibuffer_pos(new uint8_t[file_size_pos]);
  ifs_pos.read((char*)ibuffer_pos.get(), file_size_pos);
  ros::serialization::IStream istream_pos(ibuffer_pos.get(), file_size_pos);
  ros::serialization::deserialize(istream_pos, msg_pos);
  ifs_pos.close();

  for (int i = 0; i < msg_pos.poses.size(); i++)
  {
    tx_vector.push_back(msg_pos.poses[i].position.x);
    ty_vector.push_back(msg_pos.poses[i].position.y);
    tz_vector.push_back(msg_pos.poses[i].position.z);
    ta_vector.push_back(msg_pos.poses[i].orientation.x);
    wp_vector.push_back(msg_pos.poses[i].orientation.y);
  }

  for (uint32_t i = 0; i < wp_vector.size(); ++i)
  {
    wp_test.push_back(_sut.computeCloudWeight(points_grid, tx_vector[i], ty_vector[i], tz_vector[i], 0, 0, ta_vector[i]));
    wp_test[i] /= wtp;
    EXPECT_NEAR(wp_vector[i], wp_test[i], 0.0001);
  }
}

TEST_F(Grid3dTest, isIntoMapTest)
{
  // In mapfile_complete: x_max_ = 26.1 y_max_ = 19.2 z_max_ = 7.65
  float x_into = 1;
  float y_into = 1;
  float z_into = 1;
  float x_ninto = -100;
  float y_ninto = -100;
  float z_ninto = -100;

  std::string source = PROJECT_SOURCE_DIR;
  std::string map_path = source + "/data/map/mapfile_complete.bt";

  bool open = _sut.open(map_path, DEFAULT_SENSOR_DEV);

  bool isInto = _sut.isIntoMap(x_into, y_into, z_into);
  bool isNotInto = _sut.isIntoMap(x_ninto, y_ninto, z_ninto);

  EXPECT_EQ(open, true);
  EXPECT_EQ(isInto, true);
  EXPECT_EQ(isNotInto, false);
}
