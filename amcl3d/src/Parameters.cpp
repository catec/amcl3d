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
  if (!ros::param::get("~inCloudTopic", inCloudTopic))
  {
    exitWithParameterError("inCloudTopic");
  }

  if (!ros::param::get("~inOdomTopic", inOdomTopic))
  {
    exitWithParameterError("inOdomTopic");
  }

  if (!ros::param::get("~inRangeTopic", inRangeTopic))
  {
    exitWithParameterError("inRangeTopic");
  }
  if (!ros::param::get("~baseFrameId_", baseFrameId_))
  {
    exitWithParameterError("baseFrameId_");
  }

  if (!ros::param::get("~odomFrameId_", odomFrameId_))
  {
    exitWithParameterError("odomFrameId_");
  }

  if (!ros::param::get("~globalFrameId_", globalFrameId_))
  {
    exitWithParameterError("globalFrameId_");
  }

  if (!ros::param::get("~map_path", map_path))
  {
    exitWithParameterError("map_path");
  }

  if (!ros::param::get("~setInitialPose_", setInitialPose_))
  {
    exitWithParameterError("setInitialPose_");
  }

  if (!ros::param::get("~initX_", initX_))
  {
    exitWithParameterError("initX_");
  }

  if (!ros::param::get("~initY_", initY_))
  {
    exitWithParameterError("initY_");
  }

  if (!ros::param::get("~initZ_", initZ_))
  {
    exitWithParameterError("initZ_");
  }

  if (!ros::param::get("~initA_", initA_))
  {
    exitWithParameterError("initA_");
  }

  if (!ros::param::get("~initZOffset_", initZOffset_))
  {
    exitWithParameterError("initZOffset_");
  }

  if (!ros::param::get("~initXDev_", initXDev_))
  {
    exitWithParameterError("initXDev_");
  }

  if (!ros::param::get("~initYDev_", initYDev_))
  {
    exitWithParameterError("initYDev_");
  }

  if (!ros::param::get("~initZDev_", initZDev_))
  {
    exitWithParameterError("initZDev_");
  }

  if (!ros::param::get("~initADev_", initADev_))
  {
    exitWithParameterError("initADev_");
  }

  if (!ros::param::get("~publish_point_cloud_rate", publish_point_cloud_rate))
  {
    exitWithParameterError("publish_point_cloud_rate");
  }

  if (!ros::param::get("~grid_slice", grid_slice))
  {
    exitWithParameterError("grid_slice");
  }

  if (!ros::param::get("~publish_grid_slice_rate", publish_grid_slice_rate))
  {
    exitWithParameterError("publish_grid_slice_rate");
  }

  if (!ros::param::get("~publish_grid_tf_rate", publish_grid_tf_rate))
  {
    exitWithParameterError("publish_grid_tf_rate");
  }

  if (!ros::param::get("~sensor_dev", sensor_dev))
  {
    exitWithParameterError("sensor_dev");
  }

  if (!ros::param::get("~sensor_range", sensor_range))
  {
    exitWithParameterError("sensor_range");
  }

  if (!ros::param::get("~voxelSize_", voxelSize_))
  {
    exitWithParameterError("voxelSize_");
  }

  if (!ros::param::get("~num_particles", num_particles))
  {
    exitWithParameterError("num_particles");
  }

  if (!ros::param::get("~odomXMod_", odomXMod_))
  {
    exitWithParameterError("odomXMod_");
  }

  if (!ros::param::get("~odomYMod_", odomYMod_))
  {
    exitWithParameterError("odomYMod_");
  }

  if (!ros::param::get("~odomZMod_", odomZMod_))
  {
    exitWithParameterError("odomZMod_");
  }

  if (!ros::param::get("~odomAMod_", odomAMod_))
  {
    exitWithParameterError("odomAMod_");
  }

  if (!ros::param::get("~resampleInterval_", resampleInterval_))
  {
    exitWithParameterError("resampleInterval_");
  }

  if (!ros::param::get("~updateRate_", updateRate_))
  {
    exitWithParameterError("updateRate_");
  }

  if (!ros::param::get("~dTh_", dTh_))
  {
    exitWithParameterError("dTh_");
  }

  if (!ros::param::get("~aTh_", aTh_))
  {
    exitWithParameterError("aTh_");
  }

  if (!ros::param::get("~takeOffHeight_", takeOffHeight_))
  {
    exitWithParameterError("takeOffHeight_");
  }

  if (!ros::param::get("~alpha_", alpha_))
  {
    exitWithParameterError("alpha_");
  }

  ROS_INFO("[%s]"
           "\n   Parameters:"
           "\n      inCloudTopic=%s"
           "\n      inOdomTopic=%s"
           "\n      inRangeTopic=%s"
           "\n      baseFrameId_=%s"
           "\n      odomFrameId_=%s"
           "\n      globalFrameId_=%s"
           "\n      map_path=%s"
           "\n      setInitialPose_=%d"
           "\n      initX_=%lf"
           "\n      initY_=%lf"
           "\n      initZ_=%lf"
           "\n      initA_=%lf"
           "\n      initZOffset_=%lf"
           "\n      initXDev_=%lf"
           "\n      initYDev_=%lf"
           "\n      initZDev_=%lf"
           "\n      initADev_=%lf"
           "\n      publish_point_cloud_rate=%lf"
           "\n      grid_slice=%f"
           "\n      publish_grid_slice_rate=%lf"
           "\n      publish_grid_tf_rate=%lf"
           "\n      sensor_dev=%lf"
           "\n      sensor_range=%lf"
           "\n      voxelSize_=%lf"
           "\n      num_particles=%d"
           "\n      odomXMod_=%lf"
           "\n      odomYMod_=%lf"
           "\n      odomZMod_=%lf"
           "\n      odomAMod_=%lf"
           "\n      resampleInterval_=%d"
           "\n      updateRate_=%lf"
           "\n      dTh_=%lf"
           "\n      aTh_=%lf"
           "\n      takeOffHeight_=%lf"
           "\n      alpha_=%lf",
           ros::this_node::getName().data(), inCloudTopic.c_str(), inOdomTopic.c_str(), inRangeTopic.c_str(),
           baseFrameId_.c_str(), odomFrameId_.c_str(), globalFrameId_.c_str(), map_path.c_str(), (int)setInitialPose_,
           initX_, initY_, initZ_, initA_, initZOffset_, initXDev_, initYDev_, initZDev_, initADev_,
           publish_point_cloud_rate, grid_slice, publish_grid_slice_rate, publish_grid_tf_rate, sensor_dev,
           sensor_range, voxelSize_, num_particles, odomXMod_, odomYMod_, odomZMod_, odomAMod_, resampleInterval_,
           updateRate_, dTh_, aTh_, takeOffHeight_, alpha_);
}

void Parameters::exitWithParameterError(const char* parameterStr)
{
  ROS_ERROR("[%s] `%s` parameter not set!", ros::this_node::getName().data(), parameterStr);
  exit(EXIT_FAILURE);
}

}  // namespace amcl3d
