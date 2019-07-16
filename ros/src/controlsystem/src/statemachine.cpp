#include "statemachine.hpp"

void StateMachine::load_functors_(std::map<std::string, procedures::Procedure>& functor_map)
{
  functor_map.insert(std::make_pair(std::string("Dive"), procedures::DiveProcedure()));
}

  // Private member function used to check if a parameter.
bool StateMachine::_check_member(XmlRpc::XmlRpcValue& o, const char * m){
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
std::string StateMachine::_features_to_string(const std::size_t features)
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

void StateMachine::_read_states(XmlRpc::XmlRpcValue& state_list)
{
  // Statically load functors from procedures.hpp
  std::map<std::string, procedures::Procedure> functor_map;
  StateMachine::load_functors_(functor_map);

  // Assert proper types.
  ROS_ASSERT(state_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  states.reserve(64);

  // State list namespaces
  std::stack<std::pair<std::string,XmlRpc::XmlRpcValue>> stack;
  stack.push(std::make_pair("", state_list));

  // For each state_list, iterate over all items in that state list.
  // If the item is a state, add it to states.
  // If the item is a state_list, add it to the stack of things to process.
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

        // Determine if the element is a state.
        std::size_t features = 0b0000;
        if(_check_member(second, "procedure")) features |= 0b1000;
        if(_check_member(second, "error")) features |= 0b0100;
        if(_check_member(second, "next")) features |= 0b0010;
        if(_check_member(second, "params")) features |= 0b0001;

        // Was a valid state found.
        StateMachine::State new_state = {};
        switch(features) {
        case 0b1111: // Found state with all features, with parameters.
          // TODO add parameter support.
          new_state.has_params = true;
          new_state.params = false;
        case 0b1110: // Found state with all features, without parameters.
          // State without parameters
          new_state.name = static_cast<std::string>(it->first);
          new_state.path = path;
          try {
            // Look-up function definition.
            new_state.procedure = functor_map.at(static_cast<std::string>(second["procedure"]));
          } catch (const std::out_of_range& oor) {
            ROS_FATAL_STREAM("Invalid configuration or source. State "
                             << new_state.path << new_state.name
                             << " references functor "
                             << static_cast<std::string>(second["procedure"])
                             << " which cannot be found.");
            ROS_FATAL_STREAM("Check spelling, if it's loaded, or if it still needs to be implimented.");
            throw std::invalid_argument("Referencing non-existent functor.");
          }
          new_state.error = static_cast<std::string>(second["error"]);
          new_state.next = static_cast<std::string>(second["next"]);

          // Log state was found.
          ROS_INFO_STREAM("State [" << new_state.path << new_state.name << "]");
          states.push_back(new_state);
          break;
        case 0b0000:
          ROS_DEBUG_STREAM("State List " << static_cast<std::string>(it->first));
          stack.push(std::make_pair(path + it->first + '/', it->second));
          break;
        default:
          ROS_FATAL_STREAM("Invalid configuration provided. Found token(s) "
                           << StateMachine::_features_to_string(features)
                           << " in state which is not allowed.");
          throw std::invalid_argument("State machine configuration YAML.");
        }
      }
  }
}

void StateMachine::_gen_machine()
{
  bool found_start = false;
  bool found_end = false;
  state_index end = 65;
  for(state_index i = 0; i < states.size(); ++i)
    {
      // Using as a macro
      const State& s = states[i];
      // Need to find the start state, "dive"
      if(s.name == "dive") {
        if (found_start)
          throw std::invalid_argument("State machine configuration YAML. Multiple start (dive) states found.");
        start = i;
        found_start = true;
      }

      // And to ensure there's an end.
      if (s.name == "surface") {
        if (found_end)
          throw std::invalid_argument("State machine configuration YAML. Multiple end (surface) states found.");
        end = i;
        found_end = true;
      }

      // Add continue transition
      transitions[i][0] = i;

      // Also while building the transition table.
      bool found_next = false;
      bool found_error = false;
      for(state_index j = 0; j < states.size(); ++j)
        {
          // Using as a macro
          const State& t = states[j];

          if(s.next == (t.path + t.name)) {
            if (found_next)
              throw std::invalid_argument("State machine configuration YAML. Multiple next transitions found.");
            found_next = true;
            transitions[i][1] = j;
          } else if(s.error == (t.path + t.name)) {
            if (found_error)
              throw std::invalid_argument("State machine configuration YAML. Multiple error transitions found.");
            found_error = true;
            transitions[i][2] = j;
          }
        }
    }
  if(!found_start)
    throw std::invalid_argument("In state machine configuration YAML, start (dive) state not defined.");
  if(!found_end)
    throw std::invalid_argument("In state machine configuration YAML, end (surface) state not defined.");

  for(state_index i = 0; i < 64; ++i)
    transitions[i][3] = end;
}

// Need to pass in list of State structs,
// use that to track next state to go.
StateMachine::StateMachine(XmlRpc::XmlRpcValue& state_list) {
  _read_states(state_list);
  _gen_machine();
}
