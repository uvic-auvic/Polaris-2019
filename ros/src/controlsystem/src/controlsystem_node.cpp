#include <ros/ros.h>
#include <boost/geometry.hpp>
#include <XmlRpcValue.h>
#include <string>
#include <map>

#include "procedures.hpp"

class State {
  // State label.
  // Movement type.
  // Direction mode. (Straight, vision responsive (up/down), hydrophone responsive)
  // Direction source. (Static, vision param, hydrophone param)

  // Procedure to carry out while in this state.
  procedures::Procedure procedure;

  // Condition to Transition States.

  //
};

class StateMachine {
  
};

class ControlSystem {
public:

  using point = boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;

  struct ResultVectors {
    // Points from the camera position to detected objects.
    // The results are zero vectors if the detection is
    // off or there are no results.
    point cam_up;
    point cam_front;
    point cam_down;

    // Hydrophone assumed direction to noise source.
    point hydrophones;
  };

private:
  ros::NodeHandle& nh_;

  struct ResultVectors result_vectors_;
  StateMachine state_machine_;


public:
  ControlSystem(ros::NodeHandle& nh)
    : nh_(nh), result_vectors_{}
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
