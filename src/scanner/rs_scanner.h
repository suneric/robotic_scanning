#ifndef _S3D_RS_SCANNER_H_
#define _S3D_RS_SCANNER_H_

#include <librealsense2/rs.hpp>
#include "../util/geometry.h"

namespace s3d {
  // realsense camera
  class RSScanner
  {
  public:
    RSScanner();
    ~RSScanner();

    bool CapturePoints(rs2::points& points, rs2::frame& color);
    // bool CapturePoints(rs2::points& points, rs2::frame& color, Eigen::Affine3f& trans);
    // bool Capture();
    bool Initial();
    void Stop();
    //void RGBView();

  private:
    void DisplayImage();

  // private:
  //   bool CalculatePose(const rs2::motion_frame& gyro, const rs2::motion_frame& accel, Eigen::Affine3f& trans);
  //   Eigen::Affine3f CalculateAffine(const rs2::motion_frame& gyro, const rs2::motion_frame& accel, float dt);
  //   Eigen::Affine3f CalculateTranslation(const rs2::motion_frame& accel, float dt);

  private:
    bool m_bInitialized;
    rs2::pipeline m_pipe;
    bool m_bFirstAccel;
    double m_lastTS;
    Eigen::Vector3f m_theta;
    Eigen::Vector3f m_lastVel;
    Eigen::Affine3f m_transform;
  };

};

#endif // !_S3D_RS_SCANNER_H_
