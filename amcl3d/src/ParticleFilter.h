/*!
 * @file ParticleFilter.h
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

#include "Grid3d.h"
#include "Parameters.h"

#include <random>

#include <ros/ros.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/thread.hpp>

namespace amcl3d
{
//! Struct that contains the data concerning one particle
struct Particle
{
  //! Position
  float x, y, z;

  //! Yaw angle
  float a;

  //! Weight
  float w, wp, wr;

  Particle() : x(0), y(0), z(0), a(0), w(0), wp(0), wr(0)
  {
  }
};

//! Struct that contains the data concerning one range meaurement
struct Range
{
  float r, ax, ay, az;

  Range(const float r_, const float ax_, const float ay_, const float az_) : r(r_), ax(ax_), ay(ay_), az(az_)
  {
  }
};

class ParticleFilter
{
public:
  explicit ParticleFilter();
  virtual ~ParticleFilter();

  bool isInitialized() const
  {
    return initialized_;
  }

  //! Get mean value from particles
  Particle getMean() const
  {
    return mean_;
  }

  void buildParticlesPoseMsg(const geometry_msgs::Point32& offset, geometry_msgs::PoseArray& msg) const;

  //! Set the initial pose of the particle filter
  void init(const int num_particles, const float x_init, const float y_init, const float z_init, const float a_init,
            const float x_dev, const float y_dev, const float z_dev, const float a_dev);

  //! This function implements the PF prediction stage.
  //! Translation in X, Y and Z in meters and yaw angle incremenet in rad
  void predict(const double odom_x_mod, const double odom_y_mod, const double odom_z_mod, const double odom_a_mod,
               const double delta_x, const double delta_y, const double delta_z, const double delta_a);

  //! Update Particles with a pointcloud update
  void update(const Grid3d& grid3d, const std::vector<pcl::PointXYZ>& points, const std::vector<Range>& range_data,
              const double alpha, const double sigma);

  //! Resample the set of particles using low-variance sampling
  void resample();

private:
  float computeRangeWeight(const float x, const float y, const float z, const std::vector<Range>& range_data,
                           const double sigma);

  float ranGaussian(const double mean, const double sigma);
  float rngUniform(const float range_from, const float range_to);

  //! Indicates if the filter was initialized
  bool initialized_{ false };

  //! Particles
  std::vector<Particle> p_;
  Particle mean_;

  //! Random number generator
  std::random_device rd_;
  std::mt19937 generator_;
};

}  // namespace amcl3d
