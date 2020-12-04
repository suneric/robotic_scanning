
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#include "iiwa_trajectory.h"

using namespace s3d;
using namespace std;

Trajectory::Trajectory(const Trajectory& rhs)
{
  this->m_trajectory.assign(rhs.m_trajectory.begin(), rhs.m_trajectory.end());
}
Trajectory& Trajectory::operator = (const Trajectory& rhs)
{
  if (this != &rhs)
  {
      this->m_trajectory.assign(rhs.m_trajectory.begin(), rhs.m_trajectory.end());
  }
  return *this;
}

void Trajectory::AddPosition(const iiwa_msgs::JointPosition& jointPos)
{
  m_trajectory.push_back(jointPos);
}

int Trajectory::PositionCount() const
{
  return (int)(m_trajectory.size());
}

iiwa_msgs::JointPosition Trajectory::PositionAt(int nIndex) const
{
  return m_trajectory.at(nIndex);
}

void Trajectory::Save(const std::string& file)
{
  std::ofstream tFile(file);
  for (int i = 0; i < PositionCount(); ++i)
  {
    iiwa_msgs::JointPosition p = PositionAt(i);
    std::cout << "saving position: \n" << p.position << std::endl;
    tFile << p.position.a1 << " ";
    tFile << p.position.a2 << " ";
    tFile << p.position.a3 << " ";
    tFile << p.position.a4 << " ";
    tFile << p.position.a5 << " ";
    tFile << p.position.a6 << " ";
    tFile << p.position.a7 << "\n";
  }
  tFile.close();
}

bool Trajectory::Load(const std::string& file)
{
  std::ifstream tFile(file);
  std::string line;
  while (std::getline(tFile, line))
  {
    //std::cout << line << std::endl;
    std::stringstream linestream(line);
    iiwa_msgs::JointPosition p;
    linestream >> p.position.a1
      >> p.position.a2
      >> p.position.a3
      >> p.position.a4
      >> p.position.a5
      >> p.position.a6
      >> p.position.a7;
    //std::cout << "reading position: \n" << p.position << std::endl;
    AddPosition(p);
  }

  return true;
}
