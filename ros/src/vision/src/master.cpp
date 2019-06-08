#include <ros/ros.h>
#include "DerivedDetector.cpp"

int main(int argc, char **argv)
{
    ross::init(argc,argv, "vision");
    ross:NodeHandle nh;

    DerivedDetector dd_front("cascade_name");

    ros::Rate r(100); // CHANGE
    while(ross::ok())
    {
        CameraInput::update();

        dd_front.update();
        

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}