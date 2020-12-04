#pragma once

#include <ros/ros.h>
#include <string>
#include <mutex>
#include <thread>
#include <iostream>

namespace s3d
{
  /*
  Holder
  */
  template <typename ROSMSG>
  class Holder
  {
  public:
    Holder() = default;

    void Set(const ROSMSG& msg)
    {
      std::lock_guard<std::mutex> lock{m_mutex};
      m_data = msg;
    }

    ROSMSG Get()
    {
      std::lock_guard<std::mutex> lock{m_mutex};
      return m_data;
    }

  private:
    ROSMSG m_data;
    std::mutex m_mutex;
  };

  /*
  State
  */
  template <typename ROSMSG>
  class State
  {
  public:
    State() = default;

    void Init(const std::string& topic)
    {
      ros::NodeHandle nh;
      m_sub = nh.subscribe<ROSMSG>(topic, 1, &State<ROSMSG>::Set, this);
    }

    void Set(ROSMSG msg)
    {
      //std::cout << "State: set message." << std::endl;
      m_holder.Set(msg);
    }

    ROSMSG Get() {return m_holder.Get();}

  private:
    Holder<ROSMSG> m_holder;
    ros::Subscriber m_sub;
  };


  /*
  Command
  */
  template<typename ROSMSG>
  class Command
  {
  public:
    Command() = default;
    void Init(const std::string& topic)
    {
      ros::NodeHandle nh;
      m_pub = nh.advertise<ROSMSG>(topic, 1);
    }

    void Set(const ROSMSG& msg) {m_holder.Set(msg);}
    ROSMSG Get() {return m_holder.Get();}

    void Publish()
    {
      if (m_pub.getNumSubscribers())
      {
        m_pub.publish(Get());
      }
    }

  private:
    ros::Publisher m_pub;
    Holder<ROSMSG> m_holder;
  };

  /*
  IIWARobot, base class for command and state
  */
  class IIWARobot
  {
  public:
    virtual ~IIWARobot() = default;
    virtual void Init() = 0;

  protected:
    IIWARobot() = default;
    void InitROS(const std::string& node)
    {
        std::map<std::string, std::string> emptyArgs;
        if (!ros::isInitialized())
        {
          ros::init(emptyArgs, node, ros::init_options::AnonymousName);
        }
    }
  };

  template <typename T>
  class IIWAServices : public IIWARobot
  {
  public:
    IIWAServices() = default;
    virtual ~IIWAServices() = default;

  protected:
    virtual bool CallService() = 0;

    std::string m_name{""};
    ros::ServiceClient m_client{};
    T m_config{};
    bool m_bReady{false};
  };

};
