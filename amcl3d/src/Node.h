/*!
 * @file Node.h
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

#include "Parameters.h"
#include "ParticleFilter.h"  //! Include Grid.hpp

#include <rosinrange_msg/RangePose.h>
#include <tf/transform_broadcaster.h>

/*! \brief Namespace of the algorithm.
 */
namespace amcl3d
{
/*! \brief Class that contains all the intances of the rest of the classes.
 *
 * It is the main node of the algorithm that is responsible for its operation.
 */
class Node
{
public:
  /*! \brief Node class constructor.
   */
  explicit Node();
  /*! \brief Node class destructor.
   */
  virtual ~Node();

  /*! \brief To run the grid3d methods and subscribers/publishers.
   *
   * It runs the Grid3d::open, returning if this process has been successful. In adicction, it runs
   * Grid3d::buildGridSliceMsg and Grid3d::buildMapPointCloudMsg. Apart from what is related to Dgrid3d, it is
   * responsible for starting the subscribers and publishers, to get the information of odometry, 3d point cloud sensor
   * and radio-range sensor and to send the information of the particles, the representation of the range of radio-range
   * sensor, the algorithm result (tranformation) and the pointcloud filtered for the algorithm.
   */
  void spin();

private:
  /*! \brief To publish the environment map point cloud.
   *
   * It is the method responsible for publishing the point cloud when the timer provides the event. The timer is set by
   * the user.
   */
  void publishMapPointCloud(const ros::TimerEvent&);

  /*! \brief To publish the grid slice.
   *
   * It is the method responsible for publishing the grid slice when the timer provides the event. The timer is set by
   * the user.
   */
  void publishGridSlice(const ros::TimerEvent&);

  /*! \brief To publish the grid slice.
   *
   * It uses the methods of the ParticleFilter class, ParticleFilter::isInitialized and
   * ParticleFilter::buildParticlesPoseMsg, to publish the position of the particles.
   */
  void publishParticles();

  /*! \brief To process the point cloud that arrives from the UAV view.
   *
   * \param msg Point cloud from the UAV view.
   *
   * It makes a prediction based on odometry and updates the particles according to the previous prediction, the
   * cloud message of entry points and the range measurement of radio-range sensor. Finally, it obtains the average of
   * the particles as a result and performs the updates of the transformations and resampling of particles. To do this,
   * it uses the methods of the ParticleFilter class, ParticleFilter::predict, ParticleFilter::update, class
   * ParticleFilter::getMean and ParticleFilter::resample.
   */
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  /*! \brief To process the odometry.
   *
   * \param msg Odometry message.
   *
   * It converts odometry into a transformed form, to work with it. It checks if the filter has been initialized with
   * the initial position that is provided by the user. Subsequently, it updates the odometry transform with the
   * orientation that arrives through the message and sends it establishing a relationship between the position where
   * the UAV starts and what moves. It is also responsible for initializing the odometry and checking if it has taken
   * off to update the transforms based on this takeoff and if there is any jump in the algorithm.
   */
  void odomCallback(const geometry_msgs::TransformStampedConstPtr& msg);

  /*! \brief To process the range result of the radio-range sensors.
   *
   * \param msg Range message of radio-range sensors.
   *
   * It is responsible for grouping the positions of placement of the sensors and grouping them together with the range
   * measurement. In this way, the data is entered in the update performed in Node:pointcloudCallback, more specifically
   * in ParticleFilter:update. Also, calculates the average of the particles (estimated position of the UAV) to send all
   * the necessary data for the representation in RViz.
   */
  void rangeCallback(const rosinrange_msg::RangePoseConstPtr& msg);

  /*! \brief To check motion and time thresholds for AMCL update.
   *
   * \return <b>bool=False</b> - If there are problems with the checks.
   * \return <b>bool=True</b> - If the checks has been correct.
   *
   * The algorithm has to be updated in a shorter time than a timer indicates. If something happens that makes this
   * impossible, it goes back to the previous odometry. The same happens with the estimated translation and  rotation of
   * the UAV. It has delimited up to how much distance and rotation can change at most in the updates. This node is
   * responsible for performing these checks.
   */
  bool checkUpdateThresholds();

  /*! \brief To initialize the algorithm.
   *
   * \param init_pose Range message of radio-range sensors.
   * \param x_dev Thresholds of x-axis in initial pose.
   * \param y_dev Thresholds of y-axis in initial pose.
   * \param z_dev Thresholds of z-axis in initial pose.
   * \param a_dev Thresholds of yaw in initial pose.
   *
   * Starting from the starting pose of the UAV provided by the user. It is in charge of initializing the particles
   * taking into account the given pose and the marked deviations, to later publish them. In this way, the particles are
   * initialized around the initial pose, the threshold is marked by deviations.
   */
  void setInitialPose(const tf::Transform& init_pose, const float x_dev, const float y_dev, const float z_dev,
                      const float a_dev);

  /*! \brief Return yaw from a given TF.
   *
   * \param tf Transformation.
   * \return <b>double</b> - Yaw angle of the TF.
   *
   * It lets to know the yaw orientation of any TF.
   */
  double getYawFromTf(const tf::Transform& tf);

  /*! \brief To show range sensors in RViz.
   *
   * \param anchor_id Identification of the radio-range sensor.
   * \param r Range measurement of radio-range sensor.
   * \param uav Estimated UAV pose.
   * \param anchor Radio-range sensor pose in the environment.
   *
   * It lets show all thing relational with the radio-range sensors in RViz.
   */
  void rvizMarkerPublish(const uint32_t anchor_id, const float r, const geometry_msgs::Point& uav,
                         const geometry_msgs::Point& anchor);

  Parameters parameters_; /*!< Instance of the Parameters class */
  Grid3d grid3d_;         /*!< Instance of the Grid3d class */
  ParticleFilter pf_;     /*!< Instance of the ParticleFilter class */

  ros::NodeHandle nh_; /*!< ROS Node Handle */

  sensor_msgs::PointCloud2 map_point_cloud_msg_; /*!< Map point cloud message */
  ros::Publisher map_point_cloud_pub_;           /*!< Publisher of map point cloud message */
  ros::Timer map_point_cloud_pub_timer_;         /*!< Timer for publish map point cloud message */

  nav_msgs::OccupancyGrid grid_slice_msg_;
  ros::Publisher grid_slice_pub_;   /*!< Publisher of map grid slice message */
  ros::Timer grid_slice_pub_timer_; /*!< Timer for publish map grid slice message */

  bool is_odom_{ false };  /*!< Flag to know the initialize of odometry */
  bool amcl_out_{ false }; /*!< Flag to know jumps in algorithm */
  double roll_{ 0 };       /*!< Roll angle */
  double pitch_{ 0 };      /*!< Pitch angle */

  std::vector<Range> range_data; /*!< Vector that contains the information of radio-range sensor */
  Particle mean_p_;              /*!< Instance of the Particle struct for particles of filter */
  Particle lastmean_p_;          /*!< Instance of the Particle struct for previous update particles of filter */

  ros::Subscriber point_sub_; /*!< UAV point cloud subscriber */
  ros::Subscriber odom_sub_;  /*!< Odometry subscriber */
  ros::Subscriber range_sub_; /*!< Radio-range sensor information subscriber */

  ros::Publisher particles_pose_pub_; /*!< Particles publisher */
  ros::Publisher range_markers_pub_;  /*!< Radio-range sensor information publisher */
  ros::Publisher odom_base_pub_;      /*!< Estimated pose UAV publisher */
  ros::Publisher cloud_filter_pub_;   /*!< Filtered point cloud publisher */

  tf::Transform lastbase_2_world_tf_;         /*!< Base-world last transformation  */
  tf::Transform initodom_2_world_tf_;         /*!< Odom-world init transformation */
  tf::Transform lastodom_2_world_tf_;         /*!< Odom-world last transformation */
  tf::Transform amcl_out_lastbase_2_odom_tf_; /*!< Base-odom Transformation for the appearance of jumps */
  tf::Transform lastupdatebase_2_odom_tf_;    /*!< Base-odom last update transformation  */
  tf::Transform base_2_odom_tf_;              /*!< Base-odom Transformation */
  tf::Transform odom_increment_tf_;           /*!< Odom increase transformation  */

  ros::Time nextupdate_time_; /*!< Timer for the next update  */
};

}  // namespace amcl3d
