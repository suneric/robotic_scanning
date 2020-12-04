#pragma once

#include "iiwa_msgs/JointPosition.h"

namespace s3d {

  class Trajectory
  {
  public:
    Trajectory() = default;
    ~Trajectory() = default;

    Trajectory(const Trajectory& rhs);
    Trajectory& operator = (const Trajectory& rhs);

    void Save(const std::string& file);
    bool Load(const std::string& file);

    void AddPosition(const iiwa_msgs::JointPosition& jointPos);

    int PositionCount() const;
    iiwa_msgs::JointPosition PositionAt(int nIndex) const;

  private:
    std::vector<iiwa_msgs::JointPosition> m_trajectory;
  };

};
