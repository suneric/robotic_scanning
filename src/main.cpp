#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <boost/filesystem.hpp>

#include "scanner/rs_scanner.h"
#include "viewer/pcl_viewer.h"
#include "util/geometry.h"
#include "util/file_io.h"
#include "util/config.h"

#include "robot/state/cartesian_pose_state.h"
#include "robot/state/joint_position_state.h"
#include "robot/command/joint_position_command.h"
#include "robot/iiwa_util.h"

using namespace s3d;
using namespace std;

const string rootPath = "/home/yufeng/catkin_ws/src/robotic_scan/";

bool bStopIIWA = false;
void IIWARobotCallBack() { bStopIIWA = true;}
void MoveIIWAtoPosition(JointPositionCmd* jpCmd, const iiwa_msgs::JointPosition& pos, double tSleep)
{
  //std::cout << "send command" << std::endl;
  jpCmd->SetPosition(pos, &IIWARobotCallBack);
  int count = 0;
  while (bStopIIWA == false)
  {
    count++;
    ros::Duration(tSleep).sleep();
    if (count > 5)
    {
      // resend the command after a while if no message recieved
      //std::cout << "send command" << std::endl;
      jpCmd->SetPosition(pos, &IIWARobotCallBack);
      count = 0;
    }
  }
  bStopIIWA = false;
}

void PrintHelp();
bool UserInput();
int ParseArguments(int argc, char** argv, std::string& file, std::string& viewMesh, int& imageCount, double& timeDuration);
void ViewPointCloud(const std::string& strFile, double timeDuration);
// void Calibration(const std::string& file, int count, const Eigen::Affine3f& toolTrans, const ViewFrame& vFrame);

// main
int main(int argc, char** argv) try
{
  std::string strFile("");
  std::string dir = "";
  int imageCount = 3;
  double timeDuration = 1.0; // s
  int task = ParseArguments(argc, argv, strFile, dir, imageCount, timeDuration);

  if (task == 1) // generate trajectory
  {
    if (strFile.empty())
    {
      std::cout << "!error! == invalid file for saving trajectory" << std::endl;
      PrintHelp();
      return 0;
    }

    Trajectory t;
    JointPositionState jpState;
    ros::AsyncSpinner spinner(1); // use 4 threads
    spinner.start();
    bool bGenerateTrajecotry = true;
    while (bGenerateTrajecotry)
    {
      bGenerateTrajecotry = UserInput();
      if (!bGenerateTrajecotry) { break; }
      iiwa_msgs::JointPosition pos = jpState.Pose();
      t.AddPosition(pos);
    }
    spinner.stop();
    std::cout << "create trajectory and save to " << strFile << std::endl;
    t.Save(strFile);
    return 0;
  }
  else if (task == 2) // scanning
  {
    // check devices
    RSScanner rscam;
    if(!rscam.Initial())
    {
      std::cout << "!error! == unable to initialize camera" << std::endl;
      return -1;
    }

    // configuration
    std::cout << "setup == " << std::to_string(imageCount) << " images will be taken at each stop" << std::endl;
    std::string config = rootPath+"config/view_frame.txt";
    ViewFrame vFrame = LoadConfiguration(config);
    std::cout << "setup == load view frame (w/h/d): ["
      << vFrame.width[0] << ", " << vFrame.width[1] << ", "
      << vFrame.height[0] << ", " << vFrame.height[1] << ", "
      << vFrame.depth[0] << ", " << vFrame.depth[1] << "]\n";

    // trajectory
    Trajectory t;
    if (!strFile.empty())
    {
      t.Load(strFile);
      std::cout << "setup == load trajectory from " << strFile << std::endl;
    }
    if (t.PositionCount() == 0)
    {
      std::cout << "setup == generate default circle trajectory" << std::endl;
      t = GenerateCirclePath(90, 15);
    }

    WSPointCloudPtr finalCloud(new WSPointCloud());
    // tool transform
    Eigen::Affine3f toolTransform = ToolTransform();
    // trajecotry for capturing images
    JointPositionState jpState;
    CartesianPoseState cpState; // CartesianPoseState reader
    JointPositionCmd jpCmd; // JointPositionCommand

    ros::AsyncSpinner spinner(1); // use 1 threads
    spinner.start();
    ros::Duration(timeDuration).sleep();

    // initialize
    iiwa_msgs::JointPosition initPos = jpState.Pose();
    if (!AtHome(initPos)) // move to home
    {
      std::cout << "kuka == move to home position." << std::endl;
      iiwa_msgs::JointPosition home = JointPose(.0,.0,.0,.0,.0,.0,.0);
      MoveIIWAtoPosition(&jpCmd, home, timeDuration);
    }

    // create image along the trajectory
    for (int i = 0; i < t.PositionCount(); ++i)
    {
      std::cout << "kuka == move to " << i << " position"<< std::endl;
      iiwa_msgs::JointPosition p = t.PositionAt(i);
      MoveIIWAtoPosition(&jpCmd, p, timeDuration);

      // get camera position
      iiwa_msgs::CartesianPose cp = cpState.Pose();
      Eigen::Affine3f trans = CartesianPoseToTransform(cp);
      trans = trans*toolTransform;

      // take image
      for (int j = 0; j < imageCount; ++j)
      {
        rs2::points points;
        rs2::frame color;
        if (rscam.CapturePoints(points, color))
        {
          WSPointCloudPtr cloud = ConvertRSPointToPCLPoint(points, color);
          cloud = CropPCLPoint(cloud, vFrame);
          cloud = TransformPCLPoint(cloud, trans);
          cloud = FilterPCLPointSOR(cloud,30,1);
          cloud = FilterPCLPoint(cloud,0.005);
          *finalCloud += *cloud;
        }
      }

      std::string file = dir+"capture_"+std::to_string(i)+".ply";
      if(SaveCaptureDataPLY(file, finalCloud))
        std::cout << "data captured and save to " << file << std::endl;
    }

    // end
    iiwa_msgs::JointPosition endPos = jpState.Pose();
    if (!AtHome(endPos)) // move to home
    {
      std::cout << "kuka == move to home position" << std::endl;
      iiwa_msgs::JointPosition home = JointPose(.0,.0,.0,.0,.0,.0,.0);
      MoveIIWAtoPosition(&jpCmd, home, timeDuration);
    }

    spinner.stop();
    rscam.Stop();

    return 0;
  }
  else
  {
    PrintHelp();
    return 0;
  }
}
catch (const std::exception& e)
{
  std::cout << e.what() << std::endl;
  return -1;
}


// process user input
bool UserInput()
{
  bool setLoopFlag;
  bool inputCheck = false;
  char takeFrame;
  do
  {
    std::cout << std::endl;
    std::cout << "Record Trajecotry Position ? [y/n]";
    std::cin >> takeFrame;
    std::cout << std::endl;

    if (takeFrame == 'y' || takeFrame == 'Y')
    {
      setLoopFlag = true;
      inputCheck = true;
      takeFrame = 0;
    }
    else if (takeFrame == 'n' || takeFrame == 'N')
    {
      setLoopFlag = false;
      inputCheck = true;
      takeFrame = 0;
    }
    else
    {
      inputCheck = false;
      std::cout << "Invalid input." << std::endl;
      takeFrame = 0;
    }
  } while (inputCheck == false);

  return setLoopFlag;
}

int ParseArguments(int argc, char** argv, std::string& file, std::string& dir, int& imageCount, double& timeDuration)
{
  // 1 for creating trajectory
  // 2 for scanning
  // other will print help.
  if (argc == 1)
      return 0;

  if (argc > 1)
  {
    if (argc > 2)
      file = std::string(argv[2]);
    if (argc > 3)
      dir = std::string(argv[3]);
    if (argc > 4)
      imageCount = std::stoi(argv[4]);
    if (argc > 5)
      timeDuration = std::stod(argv[5]);

    std::string strArg(argv[1]);
    // generate trajectory
    if (strArg.compare("-gt") == 0)
    {
      return 1;
    }
    // scan
    if (strArg.compare("-scan") == 0)
    {
      return 2;
    }
  }

  return 0;
}

void PrintHelp()
{
  std::cout << "Use this app with arguments to perform robotic scanning: \n";
  std::cout << "   Create Trajectory: cmd [-gt] [trajectory_file_path(for saving)] \n";
  std::cout << "   Scan with Trajectory: cmd [-scan] [trajectory_file_path(for loading)] [output_dir] [image_count] [time_sleep] \n";
  std::cout << "   View Point Cloud: cmd [-view] [point_cloud_file]\n";
  std::cout << "   Print Help: cmd [-h|-help] \n";
  std::cout << "Note: \n if no trajectory supplied for scan, it will use a default path to demo the robotic scanning.\n";
}
