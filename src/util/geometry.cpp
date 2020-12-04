
#include <vector>
#include "geometry.h"
#include "config.h"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

using namespace s3d;
using namespace std;
using namespace pcl;



std::tuple<int, int, int> s3d::RGBTexture(const rs2::video_frame& texture, const rs2::texture_coordinate& textureXY)
{
  int width = texture.get_width();
  int height = texture.get_height();

  int xValue = std::min(std::max(int(textureXY.u*width+0.5f), 0), width-1);
  int yValue = std::min(std::max(int(textureXY.v*height+0.5f), 0), height-1);

  int bytes = xValue * texture.get_bytes_per_pixel();
  int strides = yValue * texture.get_stride_in_bytes();
  int textureIndex = (bytes + strides);

  const auto newTexture = reinterpret_cast<const uint8_t*>(texture.get_data());
  int NT1 = newTexture[textureIndex];
  int NT2 = newTexture[textureIndex+1];
  int NT3 = newTexture[textureIndex+2];

  return std::tuple<int, int, int>(NT1, NT2, NT3);
}

WSPointCloudPtr s3d::ConvertRSPointToPCLPoint(const rs2::points& points, const rs2::frame& f)
{
  if (points.size()==0)
    return nullptr;

  rs2::video_frame color = f.as<rs2::video_frame>();

  auto sp = points.get_profile().as<rs2::video_stream_profile>();
  WSPointCloudPtr cloud(new WSPointCloud());

  // config of PCL cloud object
  cloud->width = static_cast<uint32_t>(sp.width());
  cloud->height = static_cast<uint32_t>(sp.height());
  cloud->is_dense = false;
  cloud->points.resize(points.size());

  std::tuple<uint8_t, uint8_t, uint8_t> rgbColor;
  auto textureCoords = points.get_texture_coordinates();
  auto vertices = points.get_vertices();

  for (int i = 0; i < points.size(); ++i)
  {
    cloud->points[i].x = vertices[i].x;
    cloud->points[i].y = vertices[i].y;
    cloud->points[i].z = vertices[i].z;
    if (color)
    {
      rgbColor = RGBTexture(color, textureCoords[i]);
      cloud->points[i].r = std::get<0>(rgbColor);
      cloud->points[i].g = std::get<1>(rgbColor);
      cloud->points[i].b = std::get<2>(rgbColor);
    }
  }

  return cloud;
}

WSPointCloudPtr s3d::FilterPCLPoint(const WSPointCloudPtr cloud, float leafSize)
{
  if (cloud == nullptr)
    return nullptr;

  WSPointCloudPtr voxelCloud(new WSPointCloud());
  pcl::VoxelGrid<WSPoint> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leafSize, leafSize, leafSize);
  sor.filter(*voxelCloud);
  return voxelCloud;
}

WSPointCloudPtr s3d::FilterPCLPointSOR(const WSPointCloudPtr cloud, int neighbor, float thresh)
{
  if (cloud == nullptr)
    return nullptr;

  WSPointCloudPtr sorCloud(new WSPointCloud());
  pcl::StatisticalOutlierRemoval<WSPoint> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(neighbor);
  sor.setStddevMulThresh(thresh);
  sor.filter(*sorCloud);
  return sorCloud;
}

WSPointCloudPtr s3d::CropPCLPoint(const WSPointCloudPtr cloud, const ViewFrame& vFrame)
{
  if (cloud == nullptr)
    return nullptr;

  WSPointCloudPtr outputCloud(new WSPointCloud());
  pcl::CropBox<WSPoint> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(vFrame.width[0], vFrame.height[0], vFrame.depth[0], 1.0));
  boxFilter.setMax(Eigen::Vector4f(vFrame.width[1], vFrame.height[1], vFrame.depth[1], 1.0));
  boxFilter.setInputCloud(cloud);
  boxFilter.filter(*outputCloud);

  return outputCloud;
}

WSPointCloudPtr s3d::TransformPCLPoint(const WSPointCloudPtr cloud, Eigen::Affine3f transform)
{
  if (cloud == nullptr)
    return nullptr;

  pcl::PointCloud<WSPoint>::Ptr transfomedCloud(new WSPointCloud());
  pcl::transformPointCloud(*cloud, *transfomedCloud, transform);
  return transfomedCloud;
}


Eigen::Affine3f s3d::ToolTransform()
{
  // tool transform
  // center of y is left camera center which is 17.5 mm to the center line
  // start of z is 4.2 mm from outside of len
  Eigen::Affine3f toolTranslation = Eigen::Affine3f(Eigen::Translation3f(-0.03, 0.0175, 0.0388));
  // tool x-y-z fixed angles rotation
  Eigen::Affine3f rx = Eigen::Affine3f(Eigen::AngleAxisf(-0.012, Eigen::Vector3f::UnitX()));
  Eigen::Affine3f ry = Eigen::Affine3f(Eigen::AngleAxisf(0.010, Eigen::Vector3f::UnitY()));
  Eigen::Affine3f rz = Eigen::Affine3f(Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitZ()));
  Eigen::Affine3f toolRotation = rz*ry*rx;
  Eigen::Affine3f toolTransform = toolTranslation*toolRotation; // translation * rotation
  return toolTransform;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr  s3d::SmoothPCLPoint(const WSPointCloudPtr cloud, float radius)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudSmooth(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
  mls.setComputeNormals(true);
  mls.setInputCloud(cloud);
  mls.setPolynomialOrder(2);
  mls.setSearchRadius(radius);
  mls.process(*cloudSmooth);
  return cloudSmooth;
}

bool s3d::ConvertPointCloudToPolygonMesh(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud, pcl::PolygonMesh& mesh)
{
  if (nullptr == inputCloud)
    return false;

  // // Eigen::Vector4f centroid;
  // // pcl::compute3DCentroid(*cloudSmooth, centroid);
  // // ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
  // // for (int i = 0; i < cloudNormal->size(); ++i)
  // // {
  // //   cloudNormal->points[i].normal_x *= -1;
  // //   cloudNormal->points[i].normal_y *= -1;
  // //   cloudNormal->points[i].normal_z *= -1;
  // // }
  //
  // pcl::PointCloud<pcl::PointNormal>::Ptr cloudSmoothNormal(new pcl::PointCloud<pcl::PointNormal>());
  // pcl::concatenateFields(*cloudSmooth, *cloudNormal, *cloudSmoothNormal);

  // pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
  // poisson.setDepth(9);
  // poisson.setInputCloud(inputCloud);
  // poisson.reconstruct(mesh);

  // fast triangulation
  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  gp3.setSearchRadius(0.05);
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(M_PI/4);
  gp3.setMinimumAngle(M_PI/18);
  gp3.setMaximumAngle(2*M_PI/3);
  gp3.setNormalConsistency(false);
  gp3.setInputCloud(inputCloud);
  gp3.reconstruct(mesh);

  return true;
}
