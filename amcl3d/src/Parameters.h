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

  std::string inCloudTopic;
  std::string inOdomTopic;
  std::string inRangeTopic;
  std::string baseFrameId_;
  std::string odomFrameId_;
  std::string globalFrameId_;
  std::string map_path;

  bool setInitialPose_;
  double initX_, initY_, initZ_, initA_, initZOffset_;
  double initXDev_, initYDev_, initZDev_, initADev_;

  float grid_slice;
  double publish_point_cloud_rate;
  double publish_grid_slice_rate;
  double publish_grid_tf_rate;

  double sensor_dev;
  double sensor_range;
  double voxelSize_;

  int num_particles;

  double odomXMod_, odomYMod_, odomZMod_, odomAMod_;

  int resampleInterval_;

  double updateRate_;
  double dTh_, aTh_;

  double takeOffHeight_;

  double alpha_;

private:
  void exitWithParameterError(const char* parameterStr);
};

}  // namespace amcl3d
