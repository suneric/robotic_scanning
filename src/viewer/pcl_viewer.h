#ifndef _S3D_PCL_VIEWER_H_
#define _S3D_PCL_VIEWER_H_

#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PolygonMesh.h>

typedef pcl::PointXYZRGB WSPoint;
typedef pcl::PointCloud<WSPoint> WSPointCloud;
typedef WSPointCloud::Ptr WSPointCloudPtr;

namespace s3d
{

  class PCLViewer
  {
  public:
    PCLViewer(const std::string& title);
    ~PCLViewer();

    void AddCoordinate(const Eigen::Affine3f& transform, const std::string& name);
    bool IsStop() const;
    void Update(const WSPointCloudPtr cloud);
    void UpdateS(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
    void UpdateMesh(const pcl::PolygonMesh& mesh);
    void Spin(double duration = 10);

  private:
    pcl::visualization::PCLVisualizer* m_viewer;
  };

};

#endif //!_S3D_PCL_VIEWER_H_
