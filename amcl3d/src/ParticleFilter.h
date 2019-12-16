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

namespace amcl3d
{
/*! \brief Struct that contains the data concerning one particle.
 */
struct Particle
{
  float x; /*!< Position x */
  float y; /*!< Position y */
  float z; /*!< Position z */
  float a; /*!< Yaw angle */

  float w;  /*!< Total weight */
  float wp; /*!< Weight by the 3d point cloud sensor */
  float wr; /*!< Weight by the radio-range sensor*/

  Particle() : x(0), y(0), z(0), a(0), w(0), wp(0), wr(0)
  {
  }
};

/*! \brief Struct that contains the data concerning one range measurement.
 */
struct Range
{
  float r;  /*!< Range measurement */
  float ax; /*!< Position x of the sensor in the environment */
  float ay; /*!< Position y of the sensor in the environment */
  float az; /*!< Position z of the sensor in the environment */

  Range(const float r_, const float ax_, const float ay_, const float az_) : r(r_), ax(ax_), ay(ay_), az(az_)
  {
  }
};

/*! \brief Class that contains the stages of the Particle Filter.
 */
class ParticleFilter
{
public:
  /*! \brief ParticleFilter class constructor.
   */
  explicit ParticleFilter();
  /*! \brief ParticleFilter class destructor.
   */
  virtual ~ParticleFilter();

  /*! \brief To inicialite the grid map.
   *
   * \return <b>bool=False</b> - If it has not been initialized.
   * \return <b>bool=True</b> - If it has been initialized.
   *
   * It only return the variable initialized_, and this is modified in the code when the filter does the
   * ParticleFilter::init method.
   */
  bool isInitialized() const
  {
    return initialized_;
  }

  /*! \brief To get the information from the Particle struct.
   *
   * \return Particle - Particle struct.
   */
  Particle getMean() const
  {
    return mean_;
  }

  /*! \brief To build the particles pose message.
   *
   * \param msg Type of message that it is wanted to build.
   */
  void buildParticlesPoseMsg(geometry_msgs::PoseArray& msg) const;

  /*! \brief This function implements the PF init stage.
   *
   * \param num_particles Particle number in the filter.
   * \param x_init Init x-axis position.
   * \param y_init Init x-axis position.
   * \param z_init Init x-axis position.
   * \param a_init Init yaw angle orientation.
   * \param x_dev Init thresholds of x-axis position.
   * \param y_dev Init thresholds of y-axis position.
   * \param z_dev Init thresholds of z-axis position.
   * \param a_dev Init thresholds of yaw angle orientation.
   *
   * It restructures the particle vector to adapt it to the number of selected particles. Subsequently, it initializes
   * it using a Gaussian distribution and the deviation introduced. Subsequently, it calculates what would be the
   * average particle that would simulate the estimated position of the UAV.
   */
  void init(const int num_particles, const float x_init, const float y_init, const float z_init, const float a_init,
            const float x_dev, const float y_dev, const float z_dev, const float a_dev);

  /*! \brief This function implements the PF prediction stage.
   * (Translation in X, Y and Z in meters and yaw angle incremenet in rad.)
   *
   * \param odom_x_mod Increased odometry in the position of the x-axis.
   * \param odom_y_mod Increased odometry in the position of the x-axis.
   * \param odom_z_mod Increased odometry in the position of the x-axis.
   * \param odom_a_mod Increased odometry in the position of the x-axis.
   * \param delta_x Thresholds of x-axis position in prediction.
   * \param delta_y Thresholds of y-axis position in prediction.
   * \param delta_z Thresholds of z-axis position in prediction.
   * \param delta_a Thresholds of yaw angle orientation in prediction.
   *
   * It calculates the increase that has occurred in the odometry and makes predictions of where it is possible that the
   * UAV is, taking into account selected thresholds.
   */
  void predict(const double odom_x_mod, const double odom_y_mod, const double odom_z_mod, const double odom_a_mod,
               const double delta_x, const double delta_y, const double delta_z, const double delta_a);

  /*! \brief This function implements the PF update stage.
   *
   * \param grid3d Instance of the Grid3d class.
   * \param cloud Point cloud from the UAV view.
   * \param range_data Information of the radio-range sensor.
   * \param alpha Percentage weight between point cloud and range sensor.
   * \param sigma Desviation in the measurement of the radio-range sensor.
   *
   * It takes the positions of the particles to change if they are on the map. Then, it evaluates the weight of the
   * particle according to the point cloud and the measurement of the radio sensors. Finally, it normalizes the weights
   * for all particles and finds the average for the composition of the UAV pose.
   */
  void update(const Grid3d& grid3d, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
              const std::vector<Range>& range_data, const double alpha, const double sigma);

  /*! \brief This function implements the PF resample stage.
   * Translation in X, Y and Z in meters and yaw angle incremenet in rad.
   *
   * \param num_particles Particle number in the filter.
   * \param x_dev Init thresholds of x-axis position.
   * \param y_dev Init thresholds of y-axis position.
   * \param z_dev Init thresholds of z-axis position.
   * \param a_dev Init thresholds of yaw angle orientation.
   *
   * Sample the particle set again using low variance samples. So that the particles with less weights are discarded. To
   * complete the number of particles that the filter must have, new ones are introduced taking the average of those
   * that passed the resampling and applying the same variance thresholds that is applied in the prediction.
   */
  void resample();

private:
  /*! \brief To get the particle weight according to the radio-range sensor.
   *
   * \param x Position of the x-axis of the particle.
   * \param y Position of the y-axis of the particle.
   * \param z Position of the z-axis of the particle.
   * \param range_data Information of the measurement radio-range sensor.
   * \param sigma Desviation in the measurement of the radio-range sensor.
   * \return <b>float</b> - The weight of the particle according to radio-range sensor.
   *
   * It compares the values that the sensors give respect to the value according to the particles. The difference
   * between their gives differents weight for each particle.
   */
  float computeRangeWeight(const float x, const float y, const float z, const std::vector<Range>& range_data,
                           const double sigma);

  /*! \brief To generate the random value by the Gaussian distribution.
   *
   * \param mean Average of the distribution.
   * \param sigma Desviation of the distribution.
   * \return <b>float</b> - Random value.
   */
  float ranGaussian(const double mean, const double sigma);

  /*! \brief To generate the random between two values.
   *
   * \param range_from Lower end of range.
   * \param range_to Upper end of range.
   * \return <b>float</b> - Random value.
   */
  float rngUniform(const float range_from, const float range_to);

  bool initialized_{ false }; /*!< To indicate the initialition of the filter */

  std::vector<Particle> p_; /*!< Vector of particles */
  Particle mean_;           /*!< Particle to show the mean of all the particles */

  std::random_device rd_;  /*!< Random device */
  std::mt19937 generator_; /*!< Generator of random values */
};

}  // namespace amcl3d
