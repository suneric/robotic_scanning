#pragma once

#include "../iiwa_ros.h"
#include <iiwa_msgs/TimeToDestination.h>

namespace s3d {

  class TimeToDestinationService : public IIWAServices<iiwa_msgs::TimeToDestination>
  {
  public:
    TimeToDestinationService() = default;
    virtual ~TimeToDestinationService() override = default;

    void Init() override
    {
      ros::NodeHandle nh;
      m_name = "/iiwa/state/timeToDestination";
      m_client = nh.serviceClient<iiwa_msgs::TimeToDestination>(m_name);
      m_bReady = true;
    }

    double TimeToDestination()
    {
      //std::cout << "Service: TimeToDestination." << std::endl;
      if (m_bReady)
      {
        if (CallService())
          return m_timeToDestination;
        else
          return -999; // do not return -1 as it might be a meaningfull result
      }
      ROS_ERROR_STREAM("The service client was not initialized.");
    }

  protected:
    virtual bool CallService() override
    {
      if (m_client.call(m_config))
      {
        m_timeToDestination = m_config.response.remaining_time;
        // std::cout << "service - time to destination: " << m_timeToDestination << std::endl;
        return true;
      }

      return false;
    }

  private:
    double m_timeToDestination{0};
  };

};
