#include "ros/ros.h"
#include <string>
#include <map>

class State {
  // State label.
  // Movement type.
  // Direction mode. (Straight, vision responsive (up/down), hydrophone responsive)
  // Direction source. (Static, vision param, hydrophone param)
};

class StateMachine {

};

class ControlSystem {
private:
  ros::NodeHandle& nodeHandle_;

  StateMachine stateMachine_;

public:
  ControlSystem(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
  {
    // State processing.
    nodeHandle_.getParam();

  }

  int operator()() noexcept
  {
    while(ros::ok())
      {

      }
    return 0;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::NodeHandle nodeHandle("~");

  ControlSystem controlSystem(nh);

  int returnCode = controlSystem();

  ros::shutdown();
  return returnCode;
}
