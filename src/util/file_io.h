#pragma once

#include <string>
#include "geometry.h"

namespace s3d
{
  bool SaveCaptureData(const std::string& file, const WSPointCloudPtr ptCloud);
  bool LoadCaptureData(const std::string& file, WSPointCloudPtr ptCloud);

  bool SaveTransform(const std::string& file, const Eigen::Affine3f& pose);
  bool LoadTransform(const std::string& file, Eigen::Affine3f& pos);

  bool SaveCaptureDataPLY(const std::string& file, const WSPointCloudPtr ptCloud);
  bool LoadCaptureDataPLY(const std::string& file, const WSPointCloudPtr ptCloud);
};
