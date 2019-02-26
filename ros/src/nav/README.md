# Navigation Package

## Control System
This is the system responsible for controlling how the submarine moves from commands given to it from the AI package. This involves translating the directions into values that the thrusters can use, and issuing commands to the thrusters to allow the submarine to move as directed from the AI package.

### Subscriptions
This section talks about the different ROS topics the Control System is subscribed to.

#### /nav/request
The control system reads incoming control requests from this topic, and uses these updates to change how the submarine moves.

### Publishers
This section talks about the different ROS topics that the Control System offers.

#### /nav/control_parameters
This is where the current control parameters (depth, yaw_rate, forward/sideways velocity) will be published.

#### /systems/control_system
This is where general information about the status of the control system will be found.

## Other System