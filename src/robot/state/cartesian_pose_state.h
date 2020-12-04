#pragma once

#include "../iiwa_ros.h"
#include <iiwa_msgs/CartesianPose.h>

namespace s3d
{
  class CartesianPoseState : public IIWARobot
  {
  public:
    CartesianPoseState() { Init(); }

    virtual void Init() override
    {
      InitROS("CartesionPoseListener");
      m_state.Init("/iiwa/state/CartesianPose");
    }

    iiwa_msgs::CartesianPose Pose()
    {
      return m_state.Get();
    }

  private:
    State<iiwa_msgs::CartesianPose> m_state{};
  };
};
