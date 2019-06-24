#include <ros/ros.h>
#include "DerivedDetector.cpp"
#include "CameraInput.cpp"

void updateMessage(monitor::jetson_data_msg &msg);

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
        monitor::jetson_data_msg msg; // possibly move outside of while loop

        if(input.update()) {
            dd.update();
        }

        updateMessage()
        pub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

void updateMessage(monitor::jetson_data_msg &msg)
{
    msg.x_front = dd.getXF();
    msg.y_front = dd.getYF();
    msg.x_bottom = dd.getXB();
    msg.y_bottom = dd.getYB();
    msg.x_top = dd.getXT();
    msg.y_top = dd.getYT();
}