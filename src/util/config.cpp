#include <fstream>
#include <sstream>
#include "config.h"

using namespace s3d;
using namespace std;

ViewFrame s3d::LoadConfiguration(const std::string& file)
{
  ViewFrame viewFrame;

  std::ifstream tFile(file);
  std::string line;
  while (std::getline(tFile, line))
  {
    std::stringstream linestream(line);
    linestream >> viewFrame.width[0] >> viewFrame.width[1]
      >> viewFrame.height[0] >> viewFrame.height[1]
      >> viewFrame.depth[0] >> viewFrame.depth[1];
  }

  return viewFrame;
}
