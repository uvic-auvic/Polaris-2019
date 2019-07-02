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

States are dictionaries. States MUST have the following dictionary itmes fields: 'procedure' which
names some defined C++ function in header/procedures.hpp, error which names some defined path to a
state in this YAML file, and next which names some defined path to a state in this YAML file.
What is meant by 'path to state' is if a state is in the root state list it is simply labeled as it
is named, however if a state is within a state list(s) within the root state list it's label is a
list of states lists from outermost to innermost seperated by '/', with the state name separated from
the last state list by a '/' (see example below). States CAN have the following fields: params.

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
