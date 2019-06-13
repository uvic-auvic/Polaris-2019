#include "ros/ros.h"
#include <XmlRpcValue.h>
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
  ros::NodeHandle& nh_;

  StateMachine state_machine_;

public:
  controlSystem(ros::NodeHandle& nh)
    : nh_(nh)
  {

    // parameter loading examples.
    /*
    XmlRpc::XmlRpcValue v;
    nh_.param("subscribed_to_nodes", v, v);
    for(int i =0; i < v.size(); i++)
    {
      node_names_.push_back(v[i]);
      std::cerr << "node_names: " << node_names_[i] << std::endl;
    }
    */

    // code below is just a blob of crap and ideas that is looking too be far too complicated to do a simple thing.
    XmlRpc::XmlRpcValue list1;
    nh_.param("controlsystem", list1, list1);
    ROS_ASSERT(list1.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = list1.begin(); it != list1.end(); ++it)
    {
      ROS_INFO_STREAM("Found params: " << (std::string)(it->first) << " ==> " << list1[it->first]);

      XmlRpc::XmlRpcValue list2 = it->second;
      ROS_ASSERT(list2.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it2 = list2.begin();
          it2 != list2.end(); ++it2)
      {
        XmlRpc::XmlRpcValue xml_rpc_value = it2->first;

        if (params[it->first] == "config") {
          if (params[it2->first] == "pid_coeffs") {

            if (xml_rpc_value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
              continue;

            const double value = static_cast<double>(xml_rpc_value);
            ROS_INFO_STREAM(it2->first << " " << value);
          }
        }
      }
    }
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::NodeHandle nh("~");

  ControlSystem control_system();

  int operator()() noexcept
  {
    while(ros::ok())
      {
        int return_code = control_system(nh);
      }
    ros::shutdown();
    return return_code;
  }
}
