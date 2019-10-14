#include <gmock/gmock.h>

#include <Grid3d.h>

using namespace amcl3d;

class Grid3dTest : public ::testing::Test
{
protected:
  Grid3dTest() : _sut(0.05)
  {
  }

  Grid3d _sut;
};

TEST_F(Grid3dTest, openTest)
{
  std::string source = PROJECT_SOURCE_DIR;

  // Open .bt
  std::string map_path_bt = source + "/data/map/mapfile_complete.bt";
  bool open_true_bt = _sut.open(map_path_bt);
  EXPECT_EQ(open_true_bt, true);

  // Open .ot
  std::string map_path_ot = source + "/data/map/mapfile_complete_ot.ot";
  bool open_true_ot = _sut.open(map_path_ot);
  EXPECT_EQ(open_true_ot, true);

  // Open null .bt
  std::string map_path_nullbt = source + "/tests/data/mapfile_null.bt";
  bool open_false_nullbt = _sut.open(map_path_nullbt);
  EXPECT_EQ(open_false_nullbt, false);

  // Open null .ot
  std::string map_path_nullot = source + "/tests/data/mapfile_null.ot";
  bool open_false_nullot = _sut.open(map_path_nullot);
  EXPECT_EQ(open_false_nullot, false);

  // Open map_path.length() <= 3
  std::string map_path_false = ".bt";
  bool open_false = _sut.open(map_path_false);
  EXPECT_EQ(open_false, false);
}

TEST_F(Grid3dTest, buildGridSliceMsgTest)
{
  ros::Time::init();
  std::string source = PROJECT_SOURCE_DIR;
  std::string map_path = source + "/data/map/mapfile_complete.bt";

  nav_msgs::OccupancyGrid msg;
  float z_negative = -1;
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
  bool open_true = _sut.open(map_path);
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
  bool open_true = _sut.open(map_path);
  EXPECT_EQ(open_true, true);

  // cloud_ = true
  bool result_true = _sut.buildMapPointCloudMsg(msg);
  EXPECT_EQ(result_true, true);
}

TEST_F(Grid3dTest, buildGrid3d2WorldTfTest)
{
  std::string source = PROJECT_SOURCE_DIR;
  std::string map_path = source + "/data/map/mapfile_complete.bt";

  // Expect
  tf::StampedTransform tf;
  std::string global_frame_id = "tests";
  std::string child_frame_id = "grid3d";
  float p_x = -17.35;
  float p_y = -9.5;
  float p_z = -1.4;
  float r_x = 0;
  float r_y = 0;
  float r_z = 0;
  float r_w = 1;

  // Open
  bool open_true = _sut.open(map_path);
  // buildGrid3d2WorldTf
  _sut.buildGrid3d2WorldTf(global_frame_id, tf);

  float tf_p_x = tf.getOrigin().getX();
  float tf_p_y = tf.getOrigin().getY();
  float tf_p_z = tf.getOrigin().getZ();
  float tf_r_x = tf.getRotation().getX();
  float tf_r_y = tf.getRotation().getY();
  float tf_r_z = tf.getRotation().getZ();
  float tf_r_w = tf.getRotation().getW();

  EXPECT_EQ(tf.child_frame_id_, child_frame_id);
  EXPECT_EQ(tf.frame_id_, global_frame_id);
  EXPECT_EQ(tf_p_x, p_x);
  EXPECT_EQ(tf_p_y, p_y);
  EXPECT_EQ(tf_p_z, p_z);
  EXPECT_EQ(tf_r_x, r_x);
  EXPECT_EQ(tf_r_y, r_y);
  EXPECT_EQ(tf_r_z, r_z);
  EXPECT_EQ(tf_r_w, r_w);
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
  float wp_real = 4.32189;

  // Deserialize msg
  std::ifstream ifs(source + "/tests/data/points_grid.bin", std::ios::in | std::ios::binary);
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

  std::vector<pcl::PointXYZ> points_grid;
  pcl::PointXYZ point_grid;
  for (int i = 0; i < msg.poses.size(); i++)
  {
    point_grid.x = msg.poses[i].position.x;
    point_grid.y = msg.poses[i].position.y;
    point_grid.z = msg.poses[i].position.z;

    points_grid.push_back(point_grid);
  }

  // computeCloudWeigh without Open
  float result_0 = _sut.computeCloudWeight(points_grid, 20.017967, 10.140815, 3.372801, 0.166781);
  EXPECT_EQ(result_0, 0);

  // Open
  bool open = _sut.open(map_path);

  // computeCloudWeight in known point
  float result = _sut.computeCloudWeight(points_grid, 20.017967, 10.140815, 3.372801, 0.166781);
  EXPECT_NEAR(result, wp_real, 0.0001);
}

TEST_F(Grid3dTest, computeCloudWeightParticlesTest)
{
  std::vector<float> wp_test;
  float wtp = 3301.69;

  std::string source = PROJECT_SOURCE_DIR;
  std::string map_path = source + "/data/map/mapfile_complete.bt";

  bool open = _sut.open(map_path);

  geometry_msgs::PoseArray msg_pos;
  geometry_msgs::PoseArray msg;
  std::vector<float> tx_vector;
  std::vector<float> ty_vector;
  std::vector<float> tz_vector;
  std::vector<float> ta_vector;
  std::vector<float> wp_vector;

  // Deserialize the points
  std::ifstream ifs(source + "/tests/data/points_grid.bin", std::ios::in | std::ios::binary);
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

  std::vector<pcl::PointXYZ> points_grid;
  pcl::PointXYZ point_grid;
  for (int i = 0; i < msg.poses.size(); i++)
  {
    point_grid.x = msg.poses[i].position.x;
    point_grid.y = msg.poses[i].position.y;
    point_grid.z = msg.poses[i].position.z;

    points_grid.push_back(point_grid);
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
    wp_test.push_back(_sut.computeCloudWeight(points_grid, tx_vector[i], ty_vector[i], tz_vector[i], ta_vector[i]));
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
  float x_ninto = -1;
  float y_ninto = -1;
  float z_ninto = -1;

  std::string source = PROJECT_SOURCE_DIR;
  std::string map_path = source + "/data/map/mapfile_complete.bt";

  bool open = _sut.open(map_path);

  bool isInto = _sut.isIntoMap(x_into, y_into, z_into);
  bool isNotInto = _sut.isIntoMap(x_ninto, y_ninto, z_ninto);

  EXPECT_EQ(open, true);
  EXPECT_EQ(isInto, true);
  EXPECT_EQ(isNotInto, false);
}

TEST_F(Grid3dTest, getMinOctomapTest)
{
  float x, y, z;
  float x_min_oct = -17.35;
  float y_min_oct = -9.5;
  float z_min_oct = -1.4;

  std::string source = PROJECT_SOURCE_DIR;
  std::string map_path = source + "/data/map/mapfile_complete.bt";

  bool open = _sut.open(map_path);

  _sut.getMinOctomap(x, y, z);

  EXPECT_EQ(open, true);
  EXPECT_EQ(x, x_min_oct);
  EXPECT_EQ(y, y_min_oct);
  EXPECT_EQ(z, z_min_oct);
}