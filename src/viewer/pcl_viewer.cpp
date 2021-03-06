
#include <vector>
#include "pcl_viewer.h"
#include <pcl/common/centroid.h>

using namespace s3d;

PCLViewer::PCLViewer(const std::string& title)
{
  m_viewer = new pcl::visualization::PCLVisualizer(title);
  m_viewer->setBackgroundColor(192,192,192);
  m_viewer->addCoordinateSystem(1.0, "axis", 0);
  // camera position [camera x y z; object x, y, z; camera up]
  m_viewer->setCameraPosition(0,0,1, 1,0,0, 1,0,0);
  m_viewer->setSize(500,500);
  //m_viewer->setCameraFieldOfView(0.8);
}

PCLViewer::~PCLViewer()
{
  m_viewer->close();
  delete m_viewer;
}

bool PCLViewer::IsStop() const
{
  return m_viewer->wasStopped();
}

void PCLViewer::Update(const WSPointCloudPtr cloud)
{
  if (!m_viewer->updatePointCloud(cloud, "cloud"))
  {
    m_viewer->addPointCloud<WSPoint>(cloud, "cloud");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  }
}

void PCLViewer::UpdateS(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
  m_viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud, "cloud");
  m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  //Eigen::Vector4f centroid;
  //pcl::compute3DCentroid(*cloud, centroid);
  //m_viewer->setCameraPosition(centroid[0], centroid[1] - 1, centroid[2], 0, 0, 1);
}

void PCLViewer::UpdateMesh(const pcl::PolygonMesh& mesh)
{
  if (!m_viewer->updatePolygonMesh(mesh, "mesh"))
  {
    m_viewer->addPolygonMesh(mesh, "mesh");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mesh");
  }
}

void PCLViewer::AddCoordinate(const Eigen::Affine3f& transform, const std::string& name)
{
  m_viewer->removeCoordinateSystem(name);
  m_viewer->addCoordinateSystem(0.5, transform, name, 0);
}

void PCLViewer::Spin(double duration)
{
  m_viewer->spinOnce(duration);
}
