#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <string>
#include <map>

#include "procedures.h"

class State {
  // State label.
  // Movement type.
  // Direction mode. (Straight, vision responsive (up/down), hydrophone responsive)
  // Direction source. (Static, vision param, hydrophone param)

  // Procedure to carry out while in this state.

  // Condition to Transition States.

  //
};

class StateMachine {
  
};

class ControlSystem {
private:
  ros::NodeHandle& nh_;

  StateMachine state_machine_;

public:
  ControlSystem(ros::NodeHandle& nh)
    : nh_(nh)
  {

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
  ros::init(argc, argv, "controlsystem");
  ros::NodeHandle nh("~");

  ControlSystem control_system(nh);

  int return_code = control_system();

  ros::shutdown();
  return return_code;
}
