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

TEST_F(Grid3dTest, computeCloudWeightTest)
{
  float tx = 20.017967;
  float ty = 10.140815;
  float tz = 3.372801;
  float a = 0.166781;
  float wp_real = 4.827183;

  std::string source = PROJECT_SOURCE_DIR;
  std::string map_path = source + "/data/map/mapfile_complete.bt";

  bool open = _sut.open(map_path);

  std::ifstream points_file(source + "/tests/data/GridPoints.bin", std::ios::in | std::ios::binary);

  float num_1, num_2, num_3;
  bool first, second, third;
  int flag_1, flag_2, flag_3;

  std::vector<pcl::PointXYZ> points_read;

  if (points_file.is_open())
  {
    std::string str;
    while (std::getline(points_file, str))
    {
      if (str.size() > 0)
      {
        first = false;
        second = false;
        third = false;
        // std::cout << str << std::endl;
        for (int i = 0; i < str.size(); i++)
        {
          if (str[i] == ',' && first == false && second == false && third == false)
          {
            flag_1 = i;
            num_1 = std::stof(str.substr(1, flag_1 - 1));
            first = true;
          }
          else if (str[i] == ',' && first == true && second == false && third == false)
          {
            flag_2 = i;
            num_2 = std::stof(str.substr(flag_1 + 1, flag_2 - flag_1 - 1));
            second = true;
          }
          else if (str[i] == ')' && first == true && second == true && third == false)
          {
            flag_3 = i;
            num_3 = std::stof(str.substr(flag_2 + 1, flag_3 - flag_2 - 1));
            third = true;
          }
        }
        points_read.push_back(pcl::PointXYZ(num_1, num_2, num_3));
      }
    }
    points_file.close();
  }

  ROS_INFO("Open: [%d]", open);

  float result = _sut.computeCloudWeight(points_read, 20.017967, 10.140815, 3.372801, 0.166781);
  ROS_INFO("Result: [%lf]", result);

  EXPECT_NEAR(result, wp_real, 0.0001);
}

TEST_F(Grid3dTest, computeTestParticles)
{
  std::string source = PROJECT_SOURCE_DIR;
  std::string map_path = source + "/data/map/mapfile_complete.bt";

  bool open = _sut.open(map_path);

  std::vector<float> tx_vector;
  std::vector<float> ty_vector;
  std::vector<float> tz_vector;
  std::vector<float> ta_vector;
  std::vector<float> wp_vector;

  std::ifstream tx_file(source + "/tests/data/tx.bin", std::ios::in | std::ios::binary);

  if (tx_file.is_open())
  {
    std::string str;
    while (std::getline(tx_file, str))
    {
      if (str.size() > 0)
      {
        tx_vector.push_back(std::stof(str.substr(0, str.size())));
      }
    }
    tx_file.close();
  }

  std::ifstream ty_file(source + "/tests/data/ty.bin", std::ios::in | std::ios::binary);

  if (ty_file.is_open())
  {
    std::string str;
    while (std::getline(ty_file, str))
    {
      if (str.size() > 0)
      {
        ty_vector.push_back(std::stof(str.substr(0, str.size())));
      }
    }
    ty_file.close();
  }

  std::ifstream tz_file(source + "/tests/data/tz.bin", std::ios::in | std::ios::binary);

  if (tz_file.is_open())
  {
    std::string str;
    while (std::getline(tz_file, str))
    {
      if (str.size() > 0)
      {
        tz_vector.push_back(std::stof(str.substr(0, str.size())));
      }
    }
    tz_file.close();
  }

  std::ifstream ta_file(source + "/tests/data/ta.bin", std::ios::in | std::ios::binary);

  if (ta_file.is_open())
  {
    std::string str;
    while (std::getline(ta_file, str))
    {
      if (str.size() > 0)
      {
        ta_vector.push_back(std::stof(str.substr(0, str.size())));
      }
    }
    ta_file.close();
  }

  std::ifstream wp_file(source + "/tests/data/wp.bin", std::ios::in | std::ios::binary);

  if (wp_file.is_open())
  {
    std::string str;
    while (std::getline(wp_file, str))
    {
      if (str.size() > 0)
      {
        wp_vector.push_back(std::stof(str.substr(0, str.size())));
      }
    }
    wp_file.close();
  }

  std::ifstream points_file(source + "/tests/data/GridPoints.bin", std::ios::in | std::ios::binary);

  float num_1, num_2, num_3;
  bool first, second, third;
  int flag_1, flag_2, flag_3;

  std::vector<pcl::PointXYZ> points_read;

  if (points_file.is_open())
  {
    std::string str;
    while (std::getline(points_file, str))
    {
      if (str.size() > 0)
      {
        first = false;
        second = false;
        third = false;
        // std::cout << str << std::endl;
        for (int i = 0; i < str.size(); i++)
        {
          if (str[i] == ',' && first == false && second == false && third == false)
          {
            flag_1 = i;
            num_1 = std::stof(str.substr(1, flag_1 - 1));
            first = true;
          }
          else if (str[i] == ',' && first == true && second == false && third == false)
          {
            flag_2 = i;
            num_2 = std::stof(str.substr(flag_1 + 1, flag_2 - flag_1 - 1));
            second = true;
          }
          else if (str[i] == ')' && first == true && second == true && third == false)
          {
            flag_3 = i;
            num_3 = std::stof(str.substr(flag_2 + 1, flag_3 - flag_2 - 1));
            third = true;
          }
        }
        points_read.push_back(pcl::PointXYZ(num_1, num_2, num_3));
      }
    }
    points_file.close();
  }

  std::vector<float> wp_test;

  float wtp = 3422.208008;

  for (uint32_t i = 0; i < wp_vector.size(); ++i)
  {
    wp_test.push_back(_sut.computeCloudWeight(points_read, tx_vector[i], ty_vector[i], tz_vector[i], ta_vector[i]));
  }

  for (int i = 0; i < wp_vector.size(); i++)
  {
    wp_test[i] /= wtp;
    EXPECT_NEAR(wp_vector[i], wp_test[i], 0.0001);
    // std::cout << "OK" << std::endl;
  }
}
