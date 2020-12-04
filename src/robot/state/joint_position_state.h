#pragma once

#include "../iiwa_ros.h"
#include <iiwa_msgs/JointPosition.h>

namespace s3d
{
  class JointPositionState : public IIWARobot
  {
  public:
    JointPositionState() { Init(); }

    virtual void Init() override
    {
      InitROS("JointPositionListener");
      m_state.Init("/iiwa/state/JointPosition");
    }

    iiwa_msgs::JointPosition Pose()
    {
      return m_state.Get();
    }

  private:
    State<iiwa_msgs::JointPosition> m_state{};
  };

};
