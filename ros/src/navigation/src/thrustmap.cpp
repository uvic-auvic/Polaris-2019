#include <ros/ros.h>
#include "navigation/nav.h"
#include "navigation/thrusts.h"
#include "peripherals/rpms.h"
#include "peripherals/motor.h"
#include "peripherals/motors.h"
#include "peripherals/motor_enums.h"

#define RPM_FORWARD_SQ_COEFF    (0.00000389750967963493)
#define POWER_SCALE_FORWARD     (100.0 / sqrt(1.0 / RPM_FORWARD_SQ_COEFF))

#define RPM_REVERSE_SQ_COEFF    (0.00000396914500683942)
#define POWER_SCALE_REVERSE     (100.0 / sqrt(1.0 / RPM_REVERSE_SQ_COEFF))

#define MAX_FORWARD_COMMAND     (400) //37 original
#define MIN_FORWARD_COMMAND     (100)
#define MAX_REVERSE_COMMAND     (300) //28 original
#define MIN_REVERSE_COMMAND     (100)

#define E_MATRIX_ROWS           (8)
#define E_MATRIX_COLUMNS        (6)

#define Z_FRONT_LEFT_POS        (0)
#define Z_FRONT_RIGHT_POS       (1)
#define Z_BACK_LEFT_POS         (2)
#define Z_BACK_RIGHT_POS        (3)
#define Y_FRONT_POS	        (4)
#define Y_BACK_POS	        (5)
#define X_LEFT_POS	        (6)
#define X_RIGHT_POS	        (7)

#define VEL_X_MUL               (1.0)
#define VEL_Y_MUL               (1.0)
#define VEL_Z_MUL               (3.0)
#define ROLL_MUL                (-14.0)
#define PITCH_MUL               (-18.0)
#define YAW_MUL                 (20.0)

class thrust_controller
{
public:
    thrust_controller(std::string node_name);
    void generate_thrust_val(const navigation::nav::ConstPtr &msg);
    void do_thrust_matrix(double tau[E_MATRIX_COLUMNS], double thrust_value[]);
private:
    ros::NodeHandle nh;

    double thrust_to_motor_power(double thrust);
    double max_angular_rate;
    double max_linear_rate;
    ros::ServiceClient motors_set_all_srv;
    
    /*
    Top looking down view:
    For calculating system E only

          Front
            4
        0       1
        6       7
        2       3
            5
    */

    //motor order = 1z, 2z, 3z, 4z, 1y, 2y, 1x, 2x
    const double E_inverse[E_MATRIX_ROWS][E_MATRIX_COLUMNS] = {
        {0.0355383007316710, 0.0626175759204514, 0.226381127185469, -0.0223954134193317, -0.0186650739136927, -1.52872506320456e-17},
        {0.0355383007316709, -0.0626175759204520, 0.224903029899793, 0.0223954134193317, -0.0186650739136927, -1.61546123700340e-17},
        {-0.0355383007316710, 0.0626175759204510, 0.275096970100207, -0.0223954134193317, 0.0186650739136927, -1.68051336735253e-17},
        {-0.0355383007316710, -0.0626175759204516, 0.273618872814531, 0.0223954134193317, 0.0186650739136927, -1.03270256929244e-17},
        {0.000627995739502264, 0.475165623028772, 3.45847021392878e-17, 7.96797484263443e-17, 3.25599464924553e-18, 0.0190301739243126},
        {-0.000627995739502344, 0.524834376971227, -4.03195175199153e-17, 9.52532044474495e-17, 2.26581313390525e-18, -0.0190301739243126},
        {0.500349415164236, -0.0138177814947827, -1.91896838471135e-16, -1.39889909101611e-17, -1.04083408558608e-16, 0.0105883383101780},
        {0.499650584835764, 0.0138177814947827, 1.17507691233554e-16, -2.83689203505640e-17, -1.01047642475649e-16, -0.0105883383101780}
    };

}; // end class thrust_controller

thrust_controller::thrust_controller(std::string node_name) :
    nh(ros::NodeHandle("~"))
{
    this->nh.getParam("max_linear_vel", this->max_linear_rate);
    this->nh.getParam("max_angular_vel", this->max_angular_rate);
    this->motors_set_all_srv = nh.serviceClient<peripherals::motors>("/motor_controller/setAllMotors");
}

void thrust_controller::do_thrust_matrix(double tau[E_MATRIX_COLUMNS], double thrust_value[peripherals::motor_enums::COUNT]){
    // Thrusters = (E^-1) * tau
    for(int r = 0; r < E_MATRIX_ROWS; r++){
        thrust_value[r] = 0;
        for(int c = 0; c < E_MATRIX_COLUMNS; c++){
            thrust_value[r] += E_inverse[r][c] * tau[c];
        }
    }
}

double thrust_controller::thrust_to_motor_power(double thrust)
{
    double power = 0.0;
    if(thrust > 0)
    {
        power = sqrt(thrust / RPM_FORWARD_SQ_COEFF);
        power = POWER_SCALE_FORWARD * power;
    }
    else
    {
        power = sqrt((-thrust) / RPM_REVERSE_SQ_COEFF);
        power = POWER_SCALE_REVERSE * power;
    }

    return power;
}

void thrust_controller::generate_thrust_val(const navigation::nav::ConstPtr &msg)
{
    double tau[E_MATRIX_COLUMNS] = {
        msg->direction.x * VEL_X_MUL / this->max_linear_rate,
        msg->direction.y * VEL_Y_MUL / this->max_linear_rate,
        msg->direction.z * VEL_Z_MUL / this->max_linear_rate,
        msg->orientation.roll * ROLL_MUL / this->max_angular_rate,
        msg->orientation.pitch * PITCH_MUL / this->max_angular_rate,
        msg->orientation.yaw * YAW_MUL / this->max_angular_rate
    };   
    double thruster_vals[peripherals::motor_enums::COUNT] = {0.0};
    this->do_thrust_matrix(tau, thruster_vals);

    peripherals::motors srv;

    srv.request.X_left_power_level = this->thrust_to_motor_power(thruster_vals[X_LEFT_POS]);
    srv.request.X_right_power_level = this->thrust_to_motor_power(thruster_vals[X_RIGHT_POS]);
    srv.request.Y_front_power_level = this->thrust_to_motor_power(thruster_vals[Y_FRONT_POS]);
    srv.request.Y_rear_power_level = this->thrust_to_motor_power(thruster_vals[Y_BACK_POS]);
    srv.request.Z_front_left_power_level = this->thrust_to_motor_power(thruster_vals[Z_FRONT_LEFT_POS]);
    srv.request.Z_front_right_power_level = this->thrust_to_motor_power(thruster_vals[Z_FRONT_RIGHT_POS]);
    srv.request.Z_rear_left_power_level = this->thrust_to_motor_power(thruster_vals[Z_BACK_LEFT_POS]);
    srv.request.Z_rear_right_power_level = this->thrust_to_motor_power(thruster_vals[Z_BACK_RIGHT_POS]);

    this->motors_set_all_srv.call(srv);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thrustmap");
    ros::NodeHandle nh("~");

    // Wait until motor controller is ready
    ros::service::waitForService("/motor_controller/setAllMotors", -1);

    thrust_controller tc("motor_controller");
    ros::Subscriber joy = nh.subscribe<navigation::nav>
        ("/navigation/heading", 1, &thrust_controller::generate_thrust_val, &tc);
    

    ros::spin();

    return 0;
}
