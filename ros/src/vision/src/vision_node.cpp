#include <ros/ros.h>
#include "vision/gate.h"

#include "DerivedDetector.hpp"
#include "CameraInput.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<vision::gate>("gate", 1);


    CameraInput input;
    DerivedDetector dd(input, "cascade_name");

    ros::Rate r(10); // posibly change
    while(ros::ok())
    {
        vision::gate msg; // possibly move outside of while loop

        if(input.update()) {
            dd.update();
        }

        msg.x_front = dd.getXF();
        msg.y_front = dd.getYF();
        msg.x_bottom = dd.getXB();
        msg.y_bottom = dd.getYB();
        msg.x_top = dd.getXT();
        msg.y_top = dd.getYT();

        pub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}