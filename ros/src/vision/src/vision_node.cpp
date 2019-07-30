#include <ros/ros.h>

#include "vision/vector.h"
#include "vision/change_detection.h"

#include "EnableDetector.hpp"
#include "CameraInput.hpp"
#include "BuoyDetector.hpp"
#include "GateDetector.hpp"

// VisionSystem::EnabledDetector::NONE

class VisionSystem
{
private:
    // Ros fun stuff
    ros::NodeHandle nh_;

    ros::Publisher pub_;
    ros::ServiceServer changeDetection_;

    vision::vector msg_;

    // Image capturing and detection systems.
    CameraInput cameraInput_;
    GateDetector gateDetector_;
    BuoyDetector buoyDetector_;

    // Detectors that are Enabled
    EnabledDetector enabledDetectors_;

public:

    // CHANGE FROM VOID TO RESPONSE AND REQUEST TYPE
    bool changeDetectionHandler(vision::change_detection::Request& request, vision::change_detection::Response& response)
    {
        // Handle it.
        
        // Zero is a placeholder (grab from Request)
        enabledDetectors_ = static_cast<EnabledDetector>(request.enabled_type);
        response.enabled_type = static_cast<uint8_t>(enabledDetectors_);

        return true;
    }

    VisionSystem(ros::NodeHandle& nh) : 
        nh_(nh), 
        cameraInput_(),
        gateDetector_(cameraInput_, "../cascades/GateCascades.xml"),
        buoyDetector_(cameraInput_, ""),
        enabledDetectors_(EnabledDetector::NONE)
    {
        pub_ = nh.advertise<vision::vector>("/vision/vector", 1);
        changeDetection_ = nh.advertiseService("/vision/change_detection", &VisionSystem::changeDetectionHandler, this);
    }

    /* THIS IS THE MAIN LOOP OF THE VISION SYSTEM */
    int operator()()
    {
        int status = 0;
        
        ros::Rate r(10); // Maybe Faster
        while(ros::ok() && !status)
        {
            if(cameraInput_.update()) 
            {
                switch (enabledDetectors_)
                {
                case EnabledDetector::GATE:
                    gateDetector_.update();
                    msg_.x = gateDetector_.getXFront();
                    msg_.y = gateDetector_.getYFront();
                    msg_.z = gateDetector_.getZFront();
                    break;
                case EnabledDetector::BUOY:
                    // Use the gate detector class.
                    // buoyDetector_.update(); // not currently working
                    msg_.x = buoyDetector_.getXFront();
                    msg_.y = buoyDetector_.getYFront();
                    msg_.z = buoyDetector_.getZFront();
                    break;
                case EnabledDetector::NONE:
                default:
                    msg_.x = 0;
                    msg_.y = 0;
                    msg_.z = 0;
                    break;
                }
            }

            pub_.publish(msg_);
            ros::spinOnce();
            r.sleep();
        }
        
        return status;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle nh("~");

    VisionSystem visionSystem(nh);

    int status = visionSystem();

    ros::shutdown();
    return status;
}