//
// Created by avlec on 27/07/19.
//

class NavigationSystem {

private:
	// Ros stuffs
	ros::NodeHandle& nodeHandle_;
	ros::Publisher heading_;
	ros::ServiceServer setHeading_;
	ros::ServiceServer fullStop_;
	ros::ServiceServer controlEnable_;
	ros::Rate loopRate_;

	// Functionality related stuff.
	navigation::control_en::Request currentRequest;

public:
	bool setHeadingCallback(navigation::nav::Request& request, navigation::nav::Response& response)
	{

	}

	bool fullStopCallback(navigation::full_stop::Request& request, navigation::full_stop::Response& response)
	{

	}

	bool controlEnableCallback(navigation::control_en::Request& request, navigation::control_en::Response& response)
	{

	}

	// TODO change loop rate to use parameter provided.
	NavigationSystem(ros::NodeHandle& nh)
	: nodeHandle_(nh), loopRate_(10)
	{
		// This is replacing /nav/velocity_vectors
		heading_ = nh.advertise<navigation::nav>("/navigation/heading", 1);

		// This service is used for setting headings
		setHeading_ = nh.advertiseService<navigation::nav_request>("/navigation/set_heading", setHeadingCallback, this);

		// This service is used for bringing the submarine to a stand still.
		fullStop_ = nh.advertiseService<navigation::full_stop>("/navigation/full_stop", fullStopCallback, this);

		// This service is used for turning on/off different control parameters.
		controlEnable_ = nh.advertiseService("/navigation/control_enable", controlEnableCallback, this);
	}

	int operator()()
	{
		int status = 0;
		while(ros::ok())
		{
			// Update things.

			if (status)
				break;
		}
		return status;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigationsystem");
	ros::NodeHandle nh("~");

	NavigationSystem navigationSystem(nh);

}