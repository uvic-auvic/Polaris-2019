#include <ros/ros.h>
#include <ros/console.h>
#include <boost/geometry.hpp>
#include <XmlRpcValue.h>
#include <string>
#include <map>

#include "procedures.hpp"


/*!
  \brief This class uses procedures, to perform tasks similar to how a DFA operates.

*/
class StateMachine {
public:

  // Converting control terminology to state machine terminology.
  using symbol = procedures::Procedure::ReturnCode;
  using state = procedures::Procedure;
  using state_index = std::size_t;


private:

  std::array<state, 1> states;
  std::map<std::pair<state_index, symbol>, state_index> transitions;
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

  // The vectors stored in this struct have origin
  // at the center of the submarine.
  struct ResultVectors result_vectors_;
  StateMachine state_machine_;

  static void load_functors_(std::map<std::string, procedures::Procedure>& functor_map)
  {
    functor_map.insert(std::make_pair(std::string("Dive"), procedures::DiveProcedure()));
  }

  ControlSystem() = delete;

public:
  ControlSystem(ros::NodeHandle& nh)
    : nh_(nh), result_vectors_{}
  {
    // Statically load functors from procedures.hpp
    std::map<std::string, procedures::Procedure> functor_map;
    load_functors_(functor_map);

    XmlRpc::XmlRpcValue state_list;
    nh_.getParam("states", state_list);
    ROS_ASSERT(state_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = state_list.begin(); it != state_list.end(); ++it)
      {
        ROS_INFO_STREAM("Found state " << static_cast<std::string>(it->first) << "==>" << state_list[it->first]);
      }

  }

  int operator()() noexcept
  {
    while(ros::ok())
      {
        ros::spinOnce();
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
