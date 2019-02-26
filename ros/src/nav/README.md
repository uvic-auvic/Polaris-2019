# Navigation Package
This package is responsible for the translation of the higher-level commands that are issued in terms of: depth, yaw_rate, forward velocity, and sideways velocity. And translating that into something that can be sent to the motors.

## Control System
This is the system responsible for controlling how the submarine moves from commands given to it from the AI package. This involves translating the directions into values that the thrusters can use, and issuing commands to the thrusters to allow the submarine to move as directed from the AI package.

### Interconnectivity
This section explains all the things that this system depend on and things that other systems can use.

#### Subscribes to /nav/request
The control system reads incoming control requests from this topic, and uses these updates to change how the submarine moves.

#### Publishes to /nav/control_parameters
This is where the current control parameters (depth, yaw_rate, forward/sideways velocity) will be published.

#### Publishes to /systems/control_system
This is where general information about the status of the control system will be found.

## Other System