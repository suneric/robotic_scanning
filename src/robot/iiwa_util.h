#pragma once

#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/CartesianPose.h"
#include "iiwa_trajectory.h"

namespace s3d
{
  Eigen::Affine3f ConvertQuarternionToRotateMatrix(const Eigen::Vector4f& quarternion)
  {
    Eigen::Quaternionf q;
    q.x() = quarternion[0];
    q.y() = quarternion[1];
    q.z() = quarternion[2];
    q.w() = quarternion[3];
    Eigen::Matrix3f R = q.normalized().toRotationMatrix();
    return Eigen::Affine3f(R);
  }

  iiwa_msgs::JointPosition JointPose(float a1, float a2, float a3, float a4, float a5, float a6, float a7)
  {
    iiwa_msgs::JointPosition msg;
    msg.position.a1 = a1;
    msg.position.a2 = a2;
    msg.position.a3 = a3;
    msg.position.a4 = a4;
    msg.position.a5 = a5;
    msg.position.a6 = a6;
    msg.position.a7 = a7;
    return msg;
  }

  bool AtHome(const iiwa_msgs::JointPosition& p)
  {
    float tol = 0.001;
    if (abs(p.position.a1) > tol
    || abs(p.position.a2) > tol
    || abs(p.position.a3) > tol
    || abs(p.position.a4) > tol
    || abs(p.position.a5) > tol
    || abs(p.position.a6) > tol
    || abs(p.position.a7) > tol)
      return false;

    return true;
  }

  Eigen::Affine3f CartesianPoseToTransform(const iiwa_msgs::CartesianPose& cp)
  {
    //std::lock_guard<std::mutex> lk(m_mutex);
    geometry_msgs::Pose pose = cp.poseStamped.pose;
    float px = pose.position.x;
    float py = pose.position.y;
    float pz = pose.position.z;
    float ox = pose.orientation.x;
    float oy = pose.orientation.y;
    float oz = pose.orientation.z;
    float ow = pose.orientation.w;
    //std::cout << " position :( " << px << ", " << py << ", " << pz << ")" << std::endl;
    //std::cout << "orientation: (" << ox << ", " << oy << ", " << oz << ", " << ow << ")" << std::endl;
    Eigen::Affine3f translation = Eigen::Affine3f(Eigen::Translation3f(px, py, pz));
    Eigen::Affine3f rotation = ConvertQuarternionToRotateMatrix(Eigen::Vector4f(ox, oy, oz, ow));
    return translation*rotation; // translation * rotation
  }

  // static functions
  Trajectory GenerateCirclePath(float a6, float a3delta)
  {
    if (a6 > 110.0)
      a6 = 110.0;
    if (a6 < 0.0)
      a6 = 0.0;

    double ratio = M_PI/180;
    Trajectory circlePath;
    double a3min = -160, a3max = 160;
    for (double a3 = a3min; a3 <= a3max; a3 += a3delta)
    {
      iiwa_msgs::JointPosition p = JointPose(.0, .0, a3*ratio, .0, .0, a6*ratio, .0);
      circlePath.AddPosition(p);
    }

    return circlePath;
  }

};
