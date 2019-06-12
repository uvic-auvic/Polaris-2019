#include <ros/ros.h>
#include "DerivedDetector.cpp"

int main(int argc, char **argv)
{
    ross::init(argc, argv, "vision");
    ross:NodeHandle nh;

    DerivedDetector dd_front("cascade_name");

    ros::Rate r(10); // posibly change
    while(ross::ok())
    {
        if(dd_front.updateCameraInput()) {
            dd_front.update();
        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}