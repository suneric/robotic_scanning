#pragma once

#include "../iiwa_ros.h"
#include "../service/time_to_destination.h"

namespace s3d
{
  /*
  IIWACommand, base class for other specific command
  */
  class IIWACommand : public IIWARobot
  {
  protected:
    IIWACommand() = default;

    std::function<void()> m_callback{nullptr};
    TimeToDestinationService m_ttdService;

    void CompleteMotionWatcher()
    {
      // std::cout << "Command: CompleteMotionWatcher." << std::endl;
      if (m_callback == nullptr) {return;}

      bool flag = false;
      ros::Duration(0.1).sleep();
      while (true)
      {
        auto missingTime = m_ttdService.TimeToDestination();
        if (missingTime < -998)
          continue;

        if (missingTime > 0)
        {
          if (flag == false)
          {
            flag = true;
          }
          ros::Duration(0.5*missingTime).sleep();
        }
        else
        {
          if (flag == true)
          {
            m_callback();
            return;
          }
        }
      }
    }

  };
};
