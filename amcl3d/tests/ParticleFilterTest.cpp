#include <gmock/gmock.h>

#include <ParticleFilter.h>

using namespace amcl3d;

class ParticleFilterTest : public ::testing::Test
{
protected:
  ParticleFilterTest()
  {
  }

  ParticleFilter _sut;
};

TEST_F(ParticleFilterTest, initTest)
{
  int num_particles = 600;
  float x_init = 2.87080164308;
  float y_init = 1.63820032984;
  float z_init = 0.21460881682;
  float a_init = 0.144;
  float x_dev = 0.05;
  float y_dev = 0.05;
  float z_dev = 0.05;
  float a_dev = 0.1;

  _sut.init(num_particles, x_init, y_init, z_init, a_init, x_dev, y_dev, z_dev, a_dev);

  Particle mean_ = _sut.getMean();

  EXPECT_NEAR(mean_.x, x_init, 0.01);
  EXPECT_NEAR(mean_.y, y_init, 0.01);
  EXPECT_NEAR(mean_.z, z_init, 0.01);
  EXPECT_NEAR(mean_.a, a_init, 0.01);
}

TEST_F(ParticleFilterTest, predictTest)
{
  int num_particles = 600;
  float x_init = 2.87080164308;
  float y_init = 1.63820032984;
  float z_init = 0.21460881682;
  float a_init = 0.144;
  float x_dev = 0.05;
  float y_dev = 0.05;
  float z_dev = 0.05;
  float a_dev = 0.1;

  double odom_x_mod = 0.1;
  double odom_y_mod = 0.1;
  double odom_z_mod = 0.1;
  double odom_a_mod = 0.3;
  double delta_x = -0.067421;
  double delta_y = -0.006161;
  double delta_z = 0.130909;
  double delta_a = 0.052421;

  _sut.init(num_particles, x_init, y_init, z_init, a_init, x_dev, y_dev, z_dev, a_dev);

  Particle mean_ = _sut.getMean();

  _sut.predict(odom_x_mod, odom_y_mod, odom_z_mod, odom_a_mod, delta_x, delta_y, delta_z, delta_a);

  Particle predict_mean_ = _sut.getMean();

  EXPECT_NEAR(mean_.x, predict_mean_.x, 0.0001);
  EXPECT_NEAR(mean_.y, predict_mean_.y, 0.0001);
  EXPECT_NEAR(mean_.z, predict_mean_.z, 0.0001);
  EXPECT_NEAR(mean_.a, predict_mean_.a, 0.0001);
}
