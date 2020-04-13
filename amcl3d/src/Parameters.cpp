/*!
 * @file Parameters.cpp
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

#include "Parameters.h"

#include <ros/ros.h>

namespace amcl3d
{
Parameters::Parameters()
{
  if (!ros::param::get("~base_frame_id", base_frame_id_))
  {
    exitWithParameterError("base_frame_id");
  }

  if (!ros::param::get("~odom_frame_id", odom_frame_id_))
  {
    exitWithParameterError("odom_frame_id");
  }

  if (!ros::param::get("~global_frame_id", global_frame_id_))
  {
    exitWithParameterError("global_frame_id");
  }

  if (!ros::param::get("~map_path", map_path_))
  {
    exitWithParameterError("map_path");
  }

  if (!ros::param::get("~set_initial_pose", set_initial_pose_))
  {
    exitWithParameterError("set_initial_pose");
  }

  if (!ros::param::get("~init_x", init_x_))
  {
    exitWithParameterError("init_x");
  }

  if (!ros::param::get("~init_y", init_y_))
  {
    exitWithParameterError("init_y");
  }

  if (!ros::param::get("~init_z", init_z_))
  {
    exitWithParameterError("init_z");
  }

  if (!ros::param::get("~init_a", init_a_))
  {
    exitWithParameterError("init_a");
  }

  if (!ros::param::get("~init_x_dev", init_x_dev_))
  {
    exitWithParameterError("init_x_dev");
  }

  if (!ros::param::get("~init_y_dev", init_y_dev_))
  {
    exitWithParameterError("init_y_dev");
  }

  if (!ros::param::get("~init_z_dev", init_z_dev_))
  {
    exitWithParameterError("init_z_dev");
  }

  if (!ros::param::get("~init_a_dev", init_a_dev_))
  {
    exitWithParameterError("init_a_dev");
  }

  if (!ros::param::get("~publish_point_cloud_rate", publish_point_cloud_rate_))
  {
    exitWithParameterError("publish_point_cloud_rate");
  }

  if (!ros::param::get("~grid_slice_z", grid_slice_z_))
  {
    exitWithParameterError("grid_slice_z");
  }

  if (!ros::param::get("~publish_grid_slice_rate", publish_grid_slice_rate_))
  {
    exitWithParameterError("publish_grid_slice_rate");
  }

  if (!ros::param::get("~sensor_dev", sensor_dev_))
  {
    exitWithParameterError("sensor_dev");
  }

  if (!ros::param::get("~sensor_range", sensor_range_))
  {
    exitWithParameterError("sensor_range");
  }

  if (!ros::param::get("~voxel_size", voxel_size_))
  {
    exitWithParameterError("voxel_size");
  }

  if (!ros::param::get("~num_particles", num_particles_))
  {
    exitWithParameterError("num_particles");
  }

  if (!ros::param::get("~odom_x_mod", odom_x_mod_))
  {
    exitWithParameterError("odom_x_mod");
  }

  if (!ros::param::get("~odom_y_mod", odom_y_mod_))
  {
    exitWithParameterError("odom_y_mod");
  }

  if (!ros::param::get("~odom_z_mod", odom_z_mod_))
  {
    exitWithParameterError("odom_z_mod");
  }

  if (!ros::param::get("~odom_a_mod", odom_a_mod_))
  {
    exitWithParameterError("odom_a_mod");
  }

  if (!ros::param::get("~resample_interval", resample_interval_))
  {
    exitWithParameterError("resample_interval");
  }

  if (!ros::param::get("~update_rate", update_rate_))
  {
    exitWithParameterError("update_rate");
  }

  if (!ros::param::get("~d_th", d_th_))
  {
    exitWithParameterError("d_th");
  }

  if (!ros::param::get("~a_th", a_th_))
  {
    exitWithParameterError("a_th");
  }

  if (!ros::param::get("~take_off_height", take_off_height_))
  {
    exitWithParameterError("take_off_height");
  }

  if (!ros::param::get("~alpha", alpha_))
  {
    exitWithParameterError("alpha");
  }

  ROS_INFO("[%s]"
           "\n   Parameters:"
           "\n      base_frame_id=%s"
           "\n      odom_frame_id=%s"
           "\n      global_frame_id=%s"
           "\n      map_path=%s"
           "\n      set_initial_pose=%d"
           "\n      init_x=%lf"
           "\n      init_y=%lf"
           "\n      init_z=%lf"
           "\n      init_a=%lf"
           "\n      init_x_dev=%lf"
           "\n      init_y_dev=%lf"
           "\n      init_z_dev=%lf"
           "\n      init_a_dev=%lf"
           "\n      publish_point_cloud_rate=%lf"
           "\n      grid_slice_z=%f"
           "\n      publish_grid_slice_rate=%lf"
           "\n      sensor_dev=%lf"
           "\n      sensor_range=%lf"
           "\n      voxel_size=%lf"
           "\n      num_particles=%d"
           "\n      odom_x_mod=%lf"
           "\n      odom_y_mod=%lf"
           "\n      odom_z_mod=%lf"
           "\n      odom_a_mod=%lf"
           "\n      resample_interval=%d"
           "\n      update_rate=%lf"
           "\n      d_th=%lf"
           "\n      a_th=%lf"
           "\n      take_off_height=%lf"
           "\n      alpha=%lf",
           ros::this_node::getName().data(), base_frame_id_.c_str(), odom_frame_id_.c_str(), global_frame_id_.c_str(),
           map_path_.c_str(), (int)set_initial_pose_, init_x_, init_y_, init_z_, init_a_, init_x_dev_, init_y_dev_,
           init_z_dev_, init_a_dev_, publish_point_cloud_rate_, grid_slice_z_, publish_grid_slice_rate_, sensor_dev_,
           sensor_range_, voxel_size_, num_particles_, odom_x_mod_, odom_y_mod_, odom_z_mod_, odom_a_mod_,
           resample_interval_, update_rate_, d_th_, a_th_, take_off_height_, alpha_);
}

void Parameters::exitWithParameterError(const char* parameter_str)
{
  ROS_ERROR("[%s] `%s` parameter not set!", ros::this_node::getName().data(), parameter_str);
  exit(EXIT_FAILURE);
}

}  // namespace amcl3d
