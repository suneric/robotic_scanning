#include <fstream>
#include <sstream>
#include "file_io.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using namespace s3d;

bool s3d::SaveCaptureDataPLY(const std::string& file, const WSPointCloudPtr ptCloud)
{
  int res = pcl::io::savePLYFileASCII(file, *ptCloud);
  if (res < 0)
    return false;
  return true;
}

bool s3d::LoadCaptureDataPLY(const std::string& file, WSPointCloudPtr ptCloud)
{
  int res = pcl::io::loadPLYFile(file, *ptCloud);
  if (res < 0)
    return false;
  return true;
}

bool s3d::SaveCaptureData(const std::string& file, const WSPointCloudPtr ptCloud)
{
  int res = pcl::io::savePCDFileASCII(file, *ptCloud);
  if (res < 0)
    return false;
  return true;
}

bool s3d::LoadCaptureData(const std::string& file, WSPointCloudPtr ptCloud)
{
  int res = pcl::io::loadPCDFile(file, *ptCloud);
  if (res < 0)
    return false;
  return true;
}

bool s3d::SaveTransform(const std::string& file, const Eigen::Affine3f& pose)
{
  const float* data = pose.data();
  if (nullptr == data)
  {
    std::cout << "error - fail to get data  from transform! \n";
    return false;
  }

  std::ofstream tFile(file);
  for (int i = 0; i < 16; ++i)
    tFile << data[i] << " ";

  tFile.close();

  return true;
}

bool s3d::LoadTransform(const std::string& file, Eigen::Affine3f& pos)
{
  std::ifstream tFile(file);
  std::string line;
  while (std::getline(tFile, line))
  {
    std::stringstream linestream(line);
    float data[16];
    for (int i = 0; i < 16; ++i)
      linestream >> data[i];
    // for (int i = 0; i < 4; i++)
      // std::cout << data[i*4+0] << " " << data[i*4+1] << " " << data[i*4+2] << " " << data[i*4+3] << std::endl;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        pos.matrix()(i,j) = data[i+j*4];
  }
  return true;
}
