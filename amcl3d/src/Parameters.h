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
