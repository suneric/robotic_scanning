#ifndef _S3D_GEOMETRY_H_
#define _S3D_GEOMETRY_H_

#include <librealsense2/rs.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PolygonMesh.h>

typedef pcl::PointXYZRGB WSPoint;
typedef pcl::PointCloud<WSPoint> WSPointCloud;
typedef WSPointCloud::Ptr WSPointCloudPtr;

namespace s3d
{

  struct ViewFrame;

  std::tuple<int, int, int> RGBTexture(const rs2::video_frame& texture, const rs2::texture_coordinate& textureXY);
  WSPointCloudPtr ConvertRSPointToPCLPoint(const rs2::points& points, const rs2::frame& f);
  WSPointCloudPtr FilterPCLPoint(const WSPointCloudPtr cloud, float leafSize);
  WSPointCloudPtr CropPCLPoint(const WSPointCloudPtr cloud, const ViewFrame& vFrame);
  WSPointCloudPtr TransformPCLPoint(const WSPointCloudPtr cloud, Eigen::Affine3f transform);
  WSPointCloudPtr FilterPCLPointSOR(const WSPointCloudPtr cloud, int neighbor, float thresh);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr SmoothPCLPoint(const WSPointCloudPtr cloud, float radius);
  bool ConvertPointCloudToPolygonMesh(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, pcl::PolygonMesh& mesh);

  Eigen::Affine3f ToolTransform();

};

#endif //!_S3D_GEOMETRY_H_
