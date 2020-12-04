#pragma once

#include <string>

namespace s3d
{
  struct ViewFrame
  {
    float width[2] = {};
    float height[2] = {};
    float depth[2] = {};
  };

  ViewFrame LoadConfiguration(const std::string& file);
};
