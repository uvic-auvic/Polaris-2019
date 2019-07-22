#include <ros/ros.h>
#include <ros/console.h>
#include <boost/geometry.hpp>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <string>
#include <stack>
#include <stdexcept>

#include "statemachine.hpp"

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
    // No Z?
    point hydrophones;
  };

private:
  ros::NodeHandle& nh_;

  // The vectors stored in this struct have origin
  // at the center of the submarine.
  struct ResultVectors result_vectors_;
  //StateMachine state_machine_;


  StateMachine sm_;

public:
	ControlSystem() = delete;
  explicit ControlSystem(ros::NodeHandle& nh)
    : nh_(nh), result_vectors_{}, sm_{}
  {

    XmlRpc::XmlRpcValue state_params;
    nh_.getParam("states", state_params);
    StateMachine sm(state_params);

  }

  int operator()() noexcept
  {
  	StateMachine::StepResult step_result = StateMachine::StepResult::CONTINUE;
    while(step_result == StateMachine::StepResult::CONTINUE)
      {
    	  step_result = sm_();
        ros::spinOnce();
      }
    return static_cast<int>(step_result);
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
