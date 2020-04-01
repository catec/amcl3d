/*!
 * @file ParticleFilter.cpp
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

#include "ParticleFilter.h"

namespace amcl3d
{
ParticleFilter::ParticleFilter() : generator_(rd_())
{
}

ParticleFilter::~ParticleFilter()
{
}

void ParticleFilter::buildParticlesPoseMsg(geometry_msgs::PoseArray& msg) const
{
  msg.poses.resize(p_.size());

  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    msg.poses[i].position.x = static_cast<double>(p_[i].x);
    msg.poses[i].position.y = static_cast<double>(p_[i].y);
    msg.poses[i].position.z = static_cast<double>(p_[i].z);
    msg.poses[i].orientation.x = 0.;
    msg.poses[i].orientation.y = 0.;
    msg.poses[i].orientation.z = sin(static_cast<double>(p_[i].a * 0.5f));
    msg.poses[i].orientation.w = cos(static_cast<double>(p_[i].a * 0.5f));
  }
}

void ParticleFilter::init(const int num_particles, const float x_init, const float y_init, const float z_init,
                          const float a_init, const float x_dev, const float y_dev, const float z_dev,
                          const float a_dev)
{
  /*  Resize particle set */
  p_.resize(abs(num_particles));

  /*  Sample the given pose */
  const float dev = std::max(std::max(x_dev, y_dev), z_dev);
  const float gauss_const_1 = 1. / (dev * sqrt(2 * M_PI));
  const float gauss_const_2 = 1. / (2 * dev * dev);

  p_[0].x = x_init;
  p_[0].y = y_init;
  p_[0].z = z_init;
  p_[0].a = a_init;
  p_[0].w = gauss_const_1;

  float wt = p_[0].w;
  float dist;

  for (uint32_t i = 1; i < p_.size(); ++i)
  {
    p_[i].x = p_[0].x + ranGaussian(0, x_dev);
    p_[i].y = p_[0].y + ranGaussian(0, y_dev);
    p_[i].z = p_[0].z + ranGaussian(0, z_dev);
    p_[i].a = p_[0].a + ranGaussian(0, a_dev);

    dist = sqrt((p_[i].x - p_[0].x) * (p_[i].x - p_[0].x) + (p_[i].y - p_[0].y) * (p_[i].y - p_[0].y) +
                (p_[i].z - p_[0].z) * (p_[i].z - p_[0].z));

    p_[i].w = gauss_const_1 * exp(-dist * dist * gauss_const_2);

    wt += p_[i].w;
  }

  Particle mean_p;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    p_[i].w /= wt;

    mean_p.x += p_[i].w * p_[i].x;
    mean_p.y += p_[i].w * p_[i].y;
    mean_p.z += p_[i].w * p_[i].z;
    mean_p.a += p_[i].w * p_[i].a;
  }
  mean_ = mean_p;

  initialized_ = true;
}

void ParticleFilter::predict(const double odom_x_mod, const double odom_y_mod, const double odom_z_mod,
                             const double odom_a_mod, const double delta_x, const double delta_y, const double delta_z,
                             const double delta_a)
{
  const double x_dev = fabs(delta_x * odom_x_mod);
  const double y_dev = fabs(delta_y * odom_y_mod);
  const double z_dev = fabs(delta_z * odom_z_mod);
  const double a_dev = fabs(delta_a * odom_a_mod);

  /*  Make a prediction for all particles according to the odometry */
  float sa, ca, rand_x, rand_y;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    sa = sin(p_[i].a);
    ca = cos(p_[i].a);
    rand_x = delta_x + ranGaussian(0, x_dev);
    rand_y = delta_y + ranGaussian(0, y_dev);
    p_[i].x += ca * rand_x - sa * rand_y;
    p_[i].y += sa * rand_x + ca * rand_y;
    p_[i].z += delta_z + ranGaussian(0, z_dev);
    p_[i].a += delta_a + ranGaussian(0, a_dev);
  }
}

void ParticleFilter::update(const Grid3d& grid3d, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                            const std::vector<Range>& range_data, const double alpha, const double sigma,
                            const double roll, const double pitch)
{
  /*  Incorporate measurements */
  float wtp = 0, wtr = 0;

  clock_t begin_for1 = clock();
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    /*  Get particle information */
    float tx = p_[i].x;
    float ty = p_[i].y;
    float tz = p_[i].z;

    /*  Check the particle is into the map */
    if (!grid3d.isIntoMap(tx, ty, tz))
    {
      // std::cout << "Not into map: " << grid3d_.isIntoMap(tx, ty, tz-1.0) << std::endl;
      p_[i].w = 0;
      continue;
    }

    /*  Evaluate the weight of the point cloud */
    p_[i].wp = grid3d.computeCloudWeight(cloud, tx, ty, tz, roll, pitch, p_[i].a);

    /*  Evaluate the weight of the range sensors */
    p_[i].wr = computeRangeWeight(tx, ty, tz, range_data, sigma);

    /*  Increase the summatory of weights */
    wtp += p_[i].wp;
    wtr += p_[i].wr;
  }
  clock_t end_for1 = clock();
  double elapsed_secs = double(end_for1 - begin_for1) / CLOCKS_PER_SEC;
  ROS_DEBUG("Update time 1: [%lf] sec", elapsed_secs);

  /*  Normalize all weights */
  float wt = 0;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    if (wtp > 0)
      p_[i].wp /= wtp;
    else
      p_[i].wp = 0;

    if (wtr > 0)
      p_[i].wr /= wtr;
    else
      p_[i].wr = 0;

    if (!grid3d.isIntoMap(p_[i].x, p_[i].y, p_[i].z))
    {
      /* std::cout << "Not into map: " << grid3d_.isIntoMap(tx, ty, tz-1.0) << std::endl; */
      p_[i].w = 0;
    }
    else
      p_[i].w = p_[i].wp * alpha + p_[i].wr * (1 - alpha);
    wt += p_[i].w;
  }

  Particle mean_p;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    if (wt > 0)
      p_[i].w /= wt;
    else
      p_[i].w = 0;

    mean_p.x += p_[i].w * p_[i].x;
    mean_p.y += p_[i].w * p_[i].y;
    mean_p.z += p_[i].w * p_[i].z;
    mean_p.a += p_[i].w * p_[i].a;
  }
  mean_ = mean_p;
}

void ParticleFilter::resample()
{
  std::vector<Particle> new_p(p_.size());
  const float factor = 1.f / p_.size();
  const float r = factor * rngUniform(0, 1);
  float c = p_[0].w;
  float u;

  //! Do resamplig
  for (uint32_t m = 0, i = 0; m < p_.size(); ++m)
  {
    u = r + factor * m;
    while (u > c)
    {
      if (++i >= p_.size())
        break;
      c += p_[i].w;
    }
    new_p[m] = p_[i];
    new_p[m].w = factor;
  }

  //! Asign the new particles set
  p_ = new_p;
}

float ParticleFilter::computeRangeWeight(const float x, const float y, const float z,
                                         const std::vector<Range>& range_data, const double sigma)
{
  if (range_data.empty())
    return 0;

  float w = 1;
  const float k1 = 1.f / (sigma * sqrt(2 * M_PI));
  const float k2 = 0.5f / (sigma * sigma);
  float ax, ay, az, r;
  for (uint32_t i = 0; i < range_data.size(); ++i)
  {
    ax = range_data[i].ax;
    ay = range_data[i].ay;
    az = range_data[i].az;
    r = sqrt((x - ax) * (x - ax) + (y - ay) * (y - ay) + (z - az) * (z - az));
    w *= k1 * exp(-k2 * (r - range_data[i].r) * (r - range_data[i].r));
  }

  return w;
}

float ParticleFilter::ranGaussian(const double mean, const double sigma)
{
  std::normal_distribution<float> distribution(mean, sigma);
  return distribution(generator_);
}

float ParticleFilter::rngUniform(const float range_from, const float range_to)
{
  std::uniform_real_distribution<float> distribution(range_from, range_to);
  return distribution(generator_);
}

}  // namespace amcl3d
