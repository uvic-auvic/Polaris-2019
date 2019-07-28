#include <ros/ros.h>
#include "vision/gate.h"

#include "CameraInput.hpp"
#include "buoyDetector.cpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<vision::gate>("buoy", 1);
    vision::buoy msg_buoy; // possibly move outside of while loop

    CameraInput input;
    BuoyDetector buoy(input, "package.xml");

    ros::Rate r(10); // posibly change
    while(ros::ok())
    {

        if(input.update()) {
            buoy.update();
        }

        msg_buoy.x_front = buoy.getXFront();
        msg_buoy.y_front = buoy.getYFront();
        msg_buoy.z_front = buoy.getZFront();
        msg_buoy.x_bottom = buoy.getXBottom();
        msg_buoy.y_bottom = buoy.getYBottom();
        msg_buoy.z_bottom = buoy.getZBottom();
        msg_buoy.x_top = buoy.getXTop();
        msg_buoy.y_top = buoy.getYTop();
        msg_buoy.z_top = buoy.getZTop();

        pub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}