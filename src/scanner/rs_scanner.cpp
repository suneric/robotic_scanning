
#include <iostream>
#include "rs_scanner.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

using namespace s3d;
using namespace rs2;
using namespace std;
using namespace cv;

RSScanner::RSScanner()
{
  m_bInitialized = false;
  m_lastTS = 0;
  m_lastVel = Eigen::Vector3f(0, 0, 0);
  m_theta = Eigen::Vector3f(0, 0, 0);
}

RSScanner::~RSScanner()
{
}

void RSScanner::Stop()
{
  if (m_bInitialized)
    m_pipe.stop();
}

bool RSScanner::Initial()
{
  std::cout << "realsense == initialize camera" << std::endl;
  if (!m_bInitialized)
  {
    rs2::context ctx;
    auto list = ctx.query_devices();
    if (list.size() == 0)
      return false;

    rs2::config cfg;
    // cfg.enable_stream(RS2_STREAM_GYRO,  RS2_FORMAT_MOTION_XYZ32F);
    // cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);

    m_pipe.start(cfg);

    bool m_bFirstAccel = true;
    m_bInitialized = true;
  }

  return m_bInitialized;
}

// void RSScanner::RGBView()
// {
//   if (!m_bInitialized)
//     return;
//   cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
//   std::thread t(&RSScanner::DisplayImage, this);
//   t.detach();
//   return;
// }

void RSScanner::DisplayImage()
{
  while (true)
  {
    rs2::frameset frames = m_pipe.wait_for_frames();
    rs2::frame colorFrame = frames.get_color_frame();
    const int w = colorFrame.as<rs2::video_frame>().get_width();
    const int h = colorFrame.as<rs2::video_frame>().get_height();
    cv::Mat colorImg(cv::Size(w, h), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(colorImg, colorImg, cv::COLOR_RGB2BGR);
    cv::imshow("Image", colorImg);
    sleep(0.1);
  }
}

// bool RSScanner::Capture()
// {
//   if (!m_bInitialized)
//   {
//     std::cout << "no camera found." << std::endl;
//     return false;
//   }
//
//   try
//   {
//     rs2::frameset frameset = m_pipe.wait_for_frames();
//     if (rs2::motion_frame accelFrame = frameset.first_or_default(RS2_STREAM_ACCEL))
//     {
//       rs2_vector accelSample = accelFrame.get_motion_data();
//       std::cout << "Accel: " << accelSample.x << ", " << accelSample.y << ", " << accelSample.z << std::endl;
//     }
//     if (rs2::motion_frame gyroFrame = frameset.first_or_default(RS2_STREAM_GYRO))
//     {
//       rs2_vector gyroSample = gyroFrame.get_motion_data();
//       std::cout << "Gyro: " << gyroSample.x << ", " << gyroSample.y << ", " << gyroSample.z << std::endl;
//     }
//     if (rs2::depth_frame depthFrame = frameset.get_depth_frame())
//     {
//       float distToCenter = depthFrame.get_distance(0.5*depthFrame.get_width(), 0.5*depthFrame.get_height());
//       std::cout << "Center of camere to object is: " << distToCenter << " meters" << std::endl;
//     }
//
//     return true;
//   }
//   catch(rs2::error e)
//   {
//     std::cout << e.what() << std::endl;
//     return false;
//   }
// }

bool RSScanner::CapturePoints(rs2::points& points, rs2::frame& color)
{
  if (!m_bInitialized)
    return false;

  rs2::frameset frames = m_pipe.wait_for_frames();
  if (!frames)
    return false;

  std::cout << "realsense == take an image" << std::endl;
  rs2::pointcloud pc;
  auto fd = frames.get_depth_frame();
  auto fc = frames.get_color_frame();
  pc.map_to(fc); // map color texture to each point
  points = pc.calculate(fd);
  color = fc.as<rs2::video_frame>();

  return true;
}

// bool RSScanner::CapturePoints(rs2::points& points, rs2::frame& color, Eigen::Affine3f& transform)
// {
//   if (!m_bInitialized)
//     return false;
//
//   rs2::frameset frames = m_pipe.wait_for_frames();
//   if (!frames)
//     return false;
//
//   // calculate pose
//   auto fg = frames.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
//   rs2::motion_frame gyro = fg.as<rs2::motion_frame>();
//   auto fa = frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
//   rs2::motion_frame accel = fa.as<rs2::motion_frame>();
//   if(CalculatePose(gyro, accel, transform))
//   {
//     // calculate points
//     rs2::pointcloud pc;
//     auto fd = frames.get_depth_frame();
//     auto fc = frames.get_color_frame();
//     pc.map_to(fc); // map color texture to each point
//     points = pc.calculate(fd);
//     color = fc.as<rs2::video_frame>();
//     //std::cout << "number of points: " << points.size() << std::endl;
//   }
//
//   return true;
// }
//
// bool RSScanner::CalculatePose(const rs2::motion_frame& gyro, const rs2::motion_frame& accel, Eigen::Affine3f& trans)
// {
//   if (!gyro || !accel)
//     return false;
//
//   // delta time
//   double ts = gyro.get_timestamp();
//   double dt = (ts - m_lastTS) / 1000.0;
//   m_lastTS = ts; // update last timestamp
//   std::cout << "time change: " << dt << " s" << std::endl;
//
//   Eigen::Affine3f r = CalculateAffine(gyro, accel, dt);
//   Eigen::Affine3f t = CalculateTranslation(accel, dt);
//   Eigen::Affine3f currentTrans = t * r;
//
//   trans =  currentTrans * m_transform;
//   m_transform = trans; // update to latest transformation
//   std::cout << "transform: [" << trans(0, 0) << ", " << trans(0, 1) << ", " << trans(0, 2) << ", " << trans(0, 3) << std::endl;
//   std::cout << trans(1, 0) << ", " << trans(1, 1) << ", " << trans(1, 2) << ", " << trans(1, 3) << std::endl;
//   std::cout << trans(2, 0) << ", " << trans(2, 1) << ", " << trans(2, 2) << ", " << trans(2, 3) << std::endl;
//   std::cout << trans(3, 0) << ", " << trans(3, 1) << ", " << trans(3, 2) << ", " << trans(3, 3) << std::endl;
//   return true;
// }
//
// Eigen::Affine3f RSScanner::CalculateAffine(const rs2::motion_frame& gyro, const rs2::motion_frame& accel, float dt)
// {
//   // gyro
//   rs2_vector gv = gyro.get_motion_data();
//   // on the first iteration, use only data from accelerometer to set the camera's inital position
//   float pitch = 0, yaw = 0, roll = 0;
//   if (!m_bFirstAccel)
//   {
//     // change in gyro angle
//     pitch = (gv.x) * dt;
//     yaw = (gv.y) * dt;
//     roll =(gv.z) * dt;
//     m_theta += Eigen::Vector3f(pitch, -yaw, -roll);
//   }
//
//   // accel
//   rs2_vector av = accel.get_motion_data();
//   float R = sqrtf(av.x*av.x + av.y*av.y +av.z*av.z);
//   float accelRoll = acos(av.x/R);
//   float accelYaw = acos(av.y/R);
//   float accelPitch = acos(av.z/R);
//   if (m_bFirstAccel)
//   {
//     m_bFirstAccel = false;
//     pitch = accelPitch;
//     yaw = accelYaw;
//     roll = accelRoll;
//   }
//   else
//   {
//     // compensation filter
//     double alpha = 0.98;
//     m_theta = m_theta * alpha + Eigen::Vector3f(accelPitch, accelYaw, accelRoll) * (1-alpha);
//   }
//
//   pitch = m_theta[0];
//   yaw = m_theta[1];
//   roll = m_theta[2];
//
//   std::cout << "angle: x: " << pitch << " y: " << yaw << " z: " << roll << std::endl;
//
//   Eigen::Affine3f rx = Eigen::Affine3f(Eigen::AngleAxisf(pitch, Eigen::Vector3f(1, 0, 0)));
//   Eigen::Affine3f ry = Eigen::Affine3f(Eigen::AngleAxisf(yaw, Eigen::Vector3f(0, 1, 0)));
//   Eigen::Affine3f rz = Eigen::Affine3f(Eigen::AngleAxisf(roll, Eigen::Vector3f(0, 0, 1)));
//   return rz * ry * rx;
// }
//
// Eigen::Affine3f RSScanner::CalculateTranslation(const rs2::motion_frame& accel, float dt)
// {
//   if (m_bFirstAccel)
//     return Eigen::Affine3f();
//
//   // euler integration
//   rs2_vector av = accel.get_motion_data();
//   std::cout << "accel: x: " << av.x << " y: " << av.y << " z: " << av.z << std::endl;
//   Eigen::Vector3f acc;
//   acc[0] = av.x;
//   acc[1] = av.y;
//   acc[2] = av.z;
//   Eigen::Vector3f vel = m_lastVel + acc * dt;
//   std::cout << "vel x: " << vel[0] << " y: " << vel[1] << " z: " << vel[2] << std::endl;
//   Eigen::Vector3f dis = vel * dt + 0.5 * acc * dt * dt;
//   m_lastVel = vel;
//   std::cout<< "displacement: x: " << dis[0] << " y: " << dis[1] << " z: " << dis[2] << std::endl;
//
//   Eigen::Affine3f t = Eigen::Affine3f(Eigen::Translation3f(dis));
//   return t;
// }
