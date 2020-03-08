#include <ros/ros.h>
#include <vector>
#include <string>
#include <memory>
#include <serial/serial.h>
#include "monitor/GetSerialDevice.h"
#include "peripherals/motor.h"
#include "peripherals/motors.h"
#include "peripherals/motor_enums.h"
#include "peripherals/rpms.h"
#include "peripherals/get_motor_enums.h"
#include "serial_protocol.hpp"

#define NUM_MOTORS (8)
#define NUM_CHAR_PER_RPM (2)
#define NUM_CHAR_PER_PWM (2)
#define NUM_CHAR_PER_DIR (1)
#define MAX_MOTOR_VALUE (999)

#define X_LEFT_MULT             (-1)
#define X_RIGHT_MULT            (1)
#define Y_FRONT_MULT            (1)
#define Y_REAR_MULT             (-1)
#define Z_FRONT_RIGHT_MULT      (1)
#define Z_FRONT_LEFT_MULT       (1)
#define Z_REAR_RIGHT_MULT       (-1)
#define Z_REAR_LEFT_MULT        (1)

using MotorReq = peripherals::motor::Request;
using MotorRes = peripherals::motor::Response;
using MotorsReq = peripherals::motors::Request;
using MotorsRes = peripherals::motors::Response;
using MotorEnumsReq = peripherals::get_motor_enums::Request;
using MotorEnumsRes = peripherals::get_motor_enums::Response;
using rosserv = ros::ServiceServer;

class motor_controller {
public:
    motor_controller(const std::string & port, int baud_rate = 9600, int timeout = 1000);
    ~motor_controller();
    bool setAllMotors(MotorsReq &req, MotorsRes &res);
    bool stopAllMotors(MotorReq &, MotorRes &);
    bool getRPM(peripherals::rpms &rpms_msg);
    bool getMotorEnums(MotorEnumsReq &req, MotorEnumsRes &res);

private:
    std::unique_ptr<serial_protocol> serial_handle = nullptr;
    std::vector<double> pwm_multipliers;
};

motor_controller::motor_controller(const std::string & port, int baud_rate, int timeout) :
        pwm_multipliers(NUM_MOTORS)
{
    ROS_INFO("Connecting to motor_controller on port: %s", port.c_str());
    auto connection = std::unique_ptr<serial::Serial>(new serial::Serial(port, (u_int32_t) baud_rate, serial::Timeout::simpleTimeout(timeout)));
    this->serial_handle = std::unique_ptr<serial_protocol>(new serial_protocol(std::move(connection), PROTOCOL_NODE_MOTOR_CONTROLLER));

    pwm_multipliers[peripherals::motor_enums::X_RIGHT] = X_RIGHT_MULT;
    pwm_multipliers[peripherals::motor_enums::X_LEFT] = X_LEFT_MULT;
    pwm_multipliers[peripherals::motor_enums::Y_FRONT] = Y_FRONT_MULT;
    pwm_multipliers[peripherals::motor_enums::Y_REAR] = Y_REAR_MULT;
    pwm_multipliers[peripherals::motor_enums::Z_FRONT_LEFT] = Z_FRONT_LEFT_MULT;
    pwm_multipliers[peripherals::motor_enums::Z_FRONT_RIGHT] = Z_FRONT_RIGHT_MULT;
    pwm_multipliers[peripherals::motor_enums::Z_REAR_LEFT] = Z_REAR_LEFT_MULT;
    pwm_multipliers[peripherals::motor_enums::Z_REAR_RIGHT] = Z_REAR_RIGHT_MULT;
}

motor_controller::~motor_controller() {

}

bool motor_controller::setAllMotors(MotorsReq &req, MotorsRes &res)
{

    return true;
}

bool motor_controller::stopAllMotors(MotorReq &req, MotorRes &res)
{

    return true;
}

bool motor_controller::getRPM(peripherals::rpms &rpms_msg)
{
    bool ret = true;

    protocol_allMessages_U receivedMessage = {};
    if(serial_handle->request_data_message(PROTOCOL_MC_MESSAGE_REQUEST_MESSAGE_RPM_LOW, receivedMessage, protocol_MID_MC_motorRPMLow))
    {
        for(uint8_t motor = 0U; motor < 4U; motor++)
        {
            rpms_msg.rpms.push_back((double)receivedMessage.MC_motorRPMLow.motorSpeed[motor]);
        }

        ret &= true;
    }
    else
    {
        ret = false;
    }

    receivedMessage = {0U};
    if(serial_handle->request_data_message(PROTOCOL_MC_MESSAGE_REQUEST_MESSAGE_RPM_HIGH, receivedMessage, protocol_MID_MC_motorRPMLow))
    {
        for(uint8_t motor = 0U; motor < 4U; motor++)
        {
            rpms_msg.rpms.push_back((double)receivedMessage.MC_motorRPMHigh.motorSpeed[motor]);
        }

        ret &= true;
    }
    else
    {
        ret = false;
    }

    return ret;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "motor_con");
    ros::NodeHandle nh("~");

    monitor::GetSerialDevice srv;
    nh.getParam("device_id", srv.request.device_id);

    // Wait until serial device manager is ready
    ros::service::waitForService("/serial_manager/GetDevicePort", -1);

    ros::ServiceClient client = nh.serviceClient<monitor::GetSerialDevice>("/serial_manager/GetDevicePort");
    if (!client.call(srv))
    {
        ROS_ERROR("Couldn't get \"%s\" file descripter. Shutting down", srv.request.device_id.c_str());
        return 1;
    }

    ROS_INFO("Using Motor Controller on fd %s\n", srv.response.device_fd.c_str());

    motor_controller m(srv.response.device_fd);

    ros::Publisher rpm_pub = nh.advertise<peripherals::rpms>("MotorsRPMs", 1);

    /* Setup all the Different services/commands which we can call. Each service does its own error handling */
    rosserv stp = nh.advertiseService("stopAllMotors", &motor_controller::stopAllMotors, &m);
    rosserv sam = nh.advertiseService("setAllMotors", &motor_controller::setAllMotors, &m);

    int loop_rate;
    nh.getParam("loop_rate", loop_rate);

    ros::Rate r(loop_rate);
    while(ros::ok())
    {
        // Publish the RPMS to a topic
        peripherals::rpms rpms_msg;
        if(m.getRPM(rpms_msg))
        {
            rpm_pub.publish(rpms_msg);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
