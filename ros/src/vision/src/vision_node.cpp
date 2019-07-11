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

        msg.x_front = dd.getXFront();
        msg.y_front = dd.getYFront();
        msg.z_front = dd.getZFront();
        msg.x_bottom = dd.getXBottom();
        msg.y_bottom = dd.getYBottom();
        msg.z_bottom = dd.getZBottom();
        msg.x_top = dd.getXTop();
        msg.y_top = dd.getYTop();
        msg.z_top = dd.getZTop();

        pub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}