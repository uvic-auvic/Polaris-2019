#include <ros/ros.h>
#include "vision/vector.h"

#include "CameraInput.hpp"
#include "BuoyDetector.cpp"

// VisionSystem::EnabledDetector::NONE

class VisionSystem {
public:
    enum class EnabledDetector : int {
        NONE = 0,
        GATE = 1,
        BUOY = 2,
    };

private:
    // Ros fun stuff
    ros::NodeHandle nh_;

    ros::Publisher vector_;
    ros::ServiceServer changeDetection_;

    vision::vector msg_;

    // Image capturing and detection systems.
    CameraInput cameraInput_;
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
        response.enabled_type = enabledDetectors_;

        return true;
    }

    explict VisionSystem(ros::NodeHandle& nh)
    : nh_(nh), cameraInput_(),
      buoyDetector_(cameraInput_, "package.xml"),
      enabledDetectors_(EnabledDetector::NONE)
    {
        vector_ = nh.advertise<vision::vector>("/vision/vector", 1);
        changeDetection_ = nh.advertiseService<vision::change_detection_request, vision::change_detection_response>
            ("/vision/change_detection", &changeDetectionHandler, this);
        

    }

    /* THIS IS THE MAIN LOOP OF THE VISION SYSTEM */
    int operator()()
    {
        int status = 0;
        
        ros::Rate r(10); // Maybe Faster
        while(ros::ok() && !status)
        {
            switch (enabledDetectors)
            {
            case EnabledDetector::GATE:
                // Use the gate detector class.
                gate.update();
                front_vec.publish();
                break;
            case NONE:
                break;
            default:
                break;
            }

            if (input.update())
            {
                buoy.update();
            }

            vector.x_front = buoy.getXFront();
            vector.y_front = buoy.getYFront();
            vector.z_front = buoy.getZFront();

            pub.publish(msg);
            ros::spinOnce();
            r.sleep();
        }
        
        return status;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle nh("~");

    VisionSystem visionSystem(nh);

    int status = visionSystem();

    ros::shutdown();
    return status;
}