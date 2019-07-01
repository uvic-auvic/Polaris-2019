#include <ros/ros.h>
#include "vision/gate.h"

#include "CameraInput.hpp"
#include "DerivedDetector.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<vision::gate>("gate", 1);
    vision::gate msg; // possibly move outside of while loop

    CameraInput input;
    DerivedDetector dd(input, "package.xml");

    ros::Rate r(10); // posibly change
    while(ros::ok())
    {

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