#include <ros/ros.h>
#include <ros/console.h>
#include <boost/geometry.hpp>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <string>
#include <stack>
#include <stdexcept>

#include "procedures.hpp"


/*!
  \brief This class uses procedures, to perform tasks similar to how a DFA operates.
*/
class StateMachine {
public:
  // Converting control terminology to state machine terminology.
  
  using symbol = procedures::Procedure::ReturnCode;
  using state_index = std::size_t;

  struct State {
    procedures::Procedure procedure;
    std::string error;
    std::string next;
    bool params;

    std::string path;
    std::string name;
  };

private:
  // List of states that the state machine consists of.
  std::array<State, 64> states;
  // Transition table, rows are for transitions as follows
  // 1: next, 2: error, 3: fatal
  /*
    This is advantageuous because transitions can be performed in constant
    time, and the size_t maps directly back to states, which means states
    can be quickly. This results in the entire transition process being as
    efficient as it can be.
   */
  std::array<std::array<state_index, 64>, 3> transitions;

  static void load_functors_(std::map<std::string, procedures::Procedure>& functor_map)
  {
    functor_map.insert(std::make_pair(std::string("Dive"), procedures::DiveProcedure()));
  }

  static bool _check_member(XmlRpc::XmlRpcValue& o, const char * m){
    if(o.hasMember(m)) {
      switch(o[m].getType()) {
      case XmlRpc::XmlRpcValue::TypeString:
        return true;
      case XmlRpc::XmlRpcValue::TypeStruct:
        if(!std::strcmp(m, "params"))
          return true;
      default:
        ROS_FATAL_STREAM("Parameter parse error. " << m);
      }
    }
    return false;
  }

  // Error message function, used for converting found features
  // to a human-readable message.
  static std::string _features_to_string(const std::size_t features)
  {
    std::string result;
    if(features & 0b1000)
      result += "procedure" + ((features & 0b0111) == 0) ? "" : ", ";
    if(features & 0b0100)
      result += "error" + ((features & 0b1011) == 0) ? "" : ", ";
    if(features & 0b0010)
      result += "next" + ((features & 0b1101) == 0) ? "" : ", ";
    if(features & 0b0001)
      result += "params";
    return result;
  }

public:
  StateMachine() = default;

  // Need to pass in list of State structs,
  // use that to track next state to go.
  StateMachine(const XmlRpc::XmlRpcValue state_list) {
    // Statically load functors from procedures.hpp
    std::map<std::string, &procedures::Procedure> functor_map;
    load_functors_(functor_map);

    // Assert proper types.
    ROS_ASSERT(state_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    std::vector<State> states;
    states.reserve(64);

    // State list namespaces
    std::stack<std::pair<std::string,XmlRpc::XmlRpcValue>> stack;
    stack.push(std::make_pair("", state_list));

    // For each element in the stack.
    // NOTE elements are dynamically added.
    while(stack.size() != 0) {
      std::string path = stack.top().first;
      XmlRpc::XmlRpcValue current = stack.top().second;
      ROS_INFO_STREAM(path);
      stack.pop();

      for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = current.begin(); it != current.end(); ++it)
        {
          // Create a usable copy.
          XmlRpc::XmlRpcValue second = it->second;

          ROS_ASSERT(second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
          // ROS_INFO_STREAM("Current unit {" << static_cast<std::string>(it->first) << ',' << second.toXml() << '}');

          // Determine if the element is a state.
          std::size_t features = 0b0000;

          if(_check_member(second, "procedure")) features |= 0b1000;
          if(_check_member(second, "error")) features |= 0b0100;
          if(_check_member(second, "next")) features |= 0b0010;
          if(_check_member(second, "params")) features |= 0b0001;

          // Was a valid state found.
          State new_state = {};
          switch(features) {
          case 0b1111:
            // State with parameters
            // For now parameters don't exist.
          case 0b1110:
            {
              // State without parameters
              ROS_INFO_STREAM("State " << static_cast<std::string>(it->first));
              new_state.path = path + '/';
              new_state.procedure = functor_map[static_cast<std::string>(second["procedure"])];
              new_state.error = static_cast<std::string>(second["error"]);
              new_state.next = static_cast<std::string>(second["next"]);

              states.push_back(new_state);
            }
            break;
          case 0b0000:
            ROS_DEBUG_STREAM("State List " << static_cast<std::string>(it->first));
            stack.push(std::make_pair(path + it->first + '/', it->second));
            break;
          default:
            ROS_FATAL_STREAM("Invalid configuration provided. Found token(s) "
                             << _features_to_string(features)
                             << " in state which is not allowed.");
            throw std::invalid_argument("State machine configuration YAML.");
          }
        }
    }
  }
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

  ControlSystem() = delete;


public:
  ControlSystem(ros::NodeHandle& nh)
    : nh_(nh), result_vectors_{}, sm_{}
  {

    XmlRpc::XmlRpcValue state_params;
    nh_.getParam("states", state_params);
    StateMachine sm(state_params);
  }

  int operator()() noexcept
  {
    while(1)
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
