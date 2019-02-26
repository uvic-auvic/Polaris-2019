#include <ros/ros.h>
#include <exception>

#include <nav/request.h>

typedef struct ControlParams {
  double depth;
  double yaw_rate;
  double forwards_velocity;
  double sideways_velocity;
} ControlParams;

class ControlSystem
{
private:
  ControlSystem() { throw "ControlSystem: Invalid call to default constructor."; };
  ros::Subscriber requestUpdater;

  ros::Publisher  controlParamsPublisher;
  ros::Publisher  statusPublisher;

  ControlParams current_control_params;
  ControlParams desired;

public:
  ControlSystem(ros::NodeHandle &);
  void update();
  void requestHandler();
};

ControlSystem::ControlSystem(ros::NodeHandle & nh)
{
  // Creates a subscriber to update the control parameters.
  this->requestUpdater = nh.subscribe<nav::request>("/nav/request", 1, requestHandler, this);

  // Publishes the current control_parameters.
  this->controlParamsPublisher = nh.advertise<nav::request>("/nav/control_parameters", 1);

  // Publishes diagnostic info for the control system.
  this->statusPublisher = nh.advertise<nav::control_system_status>("/systems/control_system", 1);
}

/**
 * @brief updates the desired control parameters with values from the message.
 * 
 * @param msg new nav request
 */
void ControlSystem::requestHandler(const nav::request::ConstPtr & msg)
{
  this->desired.depth = msg.depth;
  this->desired.yaw_rate = msg.yaw_rate;
  this->desired.forwards_velocity = msg.forwards_velocity;
  this->desired.sideways_velocity = msg.sideways_velocity;
}

void ControlSystem::update() {
  // Publish current control parameters
  nav::request control_params;
  control_params.depth = current_control_params.depth;
  control_params.yaw_rate = current_control_params.yaw_rate;
  control_params.forwards_velocity = current_control_params.forwards_velocity;
  control_params.sideways_velocity = current_control_params.sideways_velocity;
  controlParamsPublisher.publish(control_params);

  // Publish current diagnostic info.
  nav::control_system_status status;
  status.readable_status = "To do.";
  status.status_code = 0;
  // TODO populate message fields here.
  statusPublisher.publish(status);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ControlSystem");

  ros::NodeHandle nh("~");

  ControlSystem controlSystem = ControlSystem(nh);

  // Update the loop at 10Hz
  ros::Rate r(10);
  while(ros::ok())
  {
    controlSystem.update();
    ros::spinOnce();
    r.sleep();
  }
}