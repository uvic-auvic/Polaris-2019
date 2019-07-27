# StateMachine configuration through YAML.

## General Understanding Behind Development

The general form of the YAML configuration file for the control systems state machine is as follows.

The root tag containing the description of the state machines states is labeled 'states'. Nested within
the root tag is a list of state lists, or actual states themselves. The reason state lists are offered
is to provide an identification system behind the tasks.

An example of why this is useful is the gate stage. Just for managing the Gate portion of the competition
there are several different stages the submarine needs to go through. Having lists of task lists
clustered under the same namespace makes understanding how the tasks are related easier. This allows the
seperation of different actions under the same general task. A YAML example of this can be seen below.

```yaml
states:

  # ...

  somestate:
    procedure: SomeProc
    error: surface
    next: buoy/locate
  buoy:
    locate:
      procedure: BuoyDetectProc
      error: surface # probably don't want to just give up
      next: buoy/orient
    orient:
      procedure: BuoyOrientProc
      params:
        side: vampire
        timeout: 45
      error: buoy/locate
      next: surface # probably want to actually proceed 

  # ...
```

## Implimentation Spec

The YAML file MUST have a root tag 'states' which is a dictionary of either all state OR state lists. To differentiate
between a state and a state list restrictions must be placed on what MUST be in a state, and what
MUST NOT be in a state list. It is also to note that the root dictionary states, would be classified
as a state list.

State lists are dictionaries containing keys being the state names, and values being the state parameters.
State lists MUST NOT use reserved words (listed below under 'State Reserved Words') or required state names
(listed below under 'Required States') as their names.

States are dictionaries. States MUST have the following dictionary item fields: 'procedure' which
names some defined C++ function in header/procedures.hpp, error which names some defined path to a
state in this YAML file, and next which names some defined path to a state in this YAML file.
What is meant by 'path to state' is if a state is in the root state list it is simply labeled as it
is named, however if a state is within a state list(s) within the root state list it's label is a
list of states lists from outermost to innermost seperated by '/', with the state name separated from
the last state list by a '/' (see example below). States CAN have the following fields, but are NOT
REQUIRED to: params. States MUST NOT use reserved words (listed below) as their names.

### State Reserved Words
- "procedure": This defines the c++ procedure that this state relies upon.
- "error": This specifies the path/name of the state this state transitions to for error behaviour.
- "next": This specifies the path/name of the state this state transitions to for normal behaviour.
- "params": This specifies a dictionary of options for the state to be created with. (This needs to be defined.)

### Required States
These states cannot be part of a state list subordinate to the root state list, as this would alter their global name.
This list is subject to change as newer and better ideas for naming and state orderings rear their ugly heads.
- "dive": This currently represents the start state for the state machine.
- "surface": This currently represents the end state for the state machine. (This will more than likely need to change.)

```yaml
states:
  startstate:
    # ...
    # my label is 'startstate'
  aaaa:
    xd:
      # ...
      # label is 'aaaa/xd'
   woah:
      finallyastate:
        # ...
        # my label is 'states/aaaa/woah/finallyastate'
```

## Possible Future Additions

An idea for possible future adaptations of the control system would be to allow the programmer and configuration
writer with the ability to specify multiple _next_ and _error_ states, in the form of a list. This could allow
for more specified behaviour from what actually results in the state switching. Which leads to less complicated
procedure routines, as they can make more specific assumptions. However, this leads to the possible need for more
procedures to be added.

# Procedures

```c++
class DerivedProcedure : public Procedure {
    DerivedProcedure()
    {
        // Possible default configuration.
    }

    // This function should be implimented this way or
    // weird things will happen.
    DerivedProcedure* clone() const override
    {
        return new DerivedProcedure(*this);
    }

    /*!
     * Provide a description of what the params are and do
     * @params:
     *  - param_name: description
     */
    virtual DerivedProcedure& operator=(XmlRpcValue& params)
    {
    
    }

    /*!
     * Provide a meaningful description of what the derived procedure does.
     * @dependencies: list all the information this functor needs to work.
     */
    Procedure::ReturnCode operator()() override
    {
        // Measure something

        // Do something from what was measured.

        // Tell the program what to do next.
        return Procedure::ReturnCode::CONTINUE;
    }
};
```