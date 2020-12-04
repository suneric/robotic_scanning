#pragma once

#include "../command/iiwa_command.h"
#include <iiwa_msgs/JointPosition.h>

namespace s3d
{
  class JointPositionCmd : public IIWACommand
  {
  public:
    JointPositionCmd() { Init();}
    void Init() override
    {
      InitROS("JointPositionPublisher");
      m_ttdService.Init();
      m_command.Init("/iiwa/command/JointPosition");
    }

    void SetPosition(const iiwa_msgs::JointPosition& position)
    {
      m_command.Set(position);
      m_command.Publish();
    }

    void SetPosition(const iiwa_msgs::JointPosition& position, const std::function<void()> callback)
    {
      // std::cout << "JointPositionCmd: SetPosition.";
      SetPosition(position);
      m_callback = callback;
      std::thread t(&JointPositionCmd::CompleteMotionWatcher, this);
      t.detach();
    }

  private:
    Command<iiwa_msgs::JointPosition> m_command{};
  };

};
