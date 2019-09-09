/*!
 * @file Parameters.h
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

#include <string>

namespace amcl3d
{
class Parameters
{
public:
  explicit Parameters();

  std::string base_frame_id_;
  std::string odom_frame_id_;
  std::string global_frame_id_;
  std::string map_path_;

  bool set_initial_pose_;
  double init_x_, init_y_, init_z_, init_a_;
  double init_x_dev_, init_y_dev_, init_z_dev_, init_a_dev_;

  float grid_slice_;
  double publish_point_cloud_rate_;
  double publish_grid_slice_rate_;
  double publish_grid_tf_rate_;

  double sensor_dev_;
  double sensor_range_;
  double voxel_size_;

  int num_particles_;

  double odom_x_mod_, odom_y_mod_, odom_z_mod_, odom_a_mod_;

  int resample_interval_;

  double update_rate_;
  double d_th_, a_th_;

  double take_off_height_;

  double alpha_;

private:
  void exitWithParameterError(const char* parameter_str);
};

}  // namespace amcl3d
