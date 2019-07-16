#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include <ros/ros.h>
#include <ros/console.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <string>
#include <stack>
#include <stdexcept>

#include "procedures.hpp"


class StateMachine {
public:
  // Converting control terminology to state machine terminology.
  using symbol = procedures::Procedure::ReturnCode;
  using state_index = std::size_t;

  // Member for verbosely describing states.
  struct State {
    procedures::Procedure procedure;
    std::string error;
    std::string next;
    bool has_params;
    bool params; // TODO add support for this.

    std::string path;
    std::string name;
  };

private:
  // List of states that the state machine consists of.
  std::vector<State> states;
  state_index start;
  // Transition table, rows are for transitions as follows
  // 0: continue, 1: next, 2: error, 3: fatal
  /*
    This is advantageuous because transitions can be performed in constant
    time, and the size_t maps directly back to states, which means states
    can be quickly. This results in the entire transition process being as
    efficient as it can be.
   */
  std::array<std::array<state_index, 64>, 4> transitions;

  static void load_functors_(std::map<std::string, procedures::Procedure>& functor_map);

  bool _check_member(XmlRpc::XmlRpcValue& o, const char* m);

  static std::string _features_to_string(const std::size_t features);

  // Used to read states from passed XmlRpcValue structure.
  void _read_states(XmlRpc::XmlRpcValue& state_list);
  // Generates transition functions in the state machine.
  void _gen_machine();

public:
  // Constructs a empty state machine.
  StateMachine() = default;
  // Constructs a state machine from a state description.
  StateMachine(XmlRpc::XmlRpcValue& state_list);
};


#endif
