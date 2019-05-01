#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

/*
  @brief Takes video from USB device, and publishes it to a ROS topic.
  This node enables the translation of the raw video data from the AUV
  cameras to the ROS topic at /video/<topic_name>, this is required to be
  variable as there are multiple cameras on the AUV.
 */
class ImagePublisher
{
private:

  ros::NodeHandle nh;

  image_transport::ImageTransport it;
  image_transport::Publisher pub;

  cv::VideoCapture source;

  ros::Rate loop_rate;

  std::string topic_name;
  std::string publisher_name;
  std::string fd;
  int fps;
  bool is_device;

  bool init_status;

public:
  ImagePublisher(ros::NodeHandle & nh):
    nh(nh), it(nh)
  {
    nh.getParam("topic_name", topic_name);
    nh.getParam("fd", fd);
    nh.getParam("fps", fps);
    nh.getParam("is_device", is_device);

    publisher_name = "/video/" + topic_name;
    it.advertise(publisher_name, 5);

    // TODO review device detection.
    if (is_device) {
      uint8_t video_index = (uint8_t) (fd.back() - '0');
      source = cv::VideoCapture(video_index);
    } else {
      source = cv::VideoCapture(fd);
    }

    if (!source.isOpened()) {
      ROS_ERROR("Failed to open device on %s", fd.c_str());
      init_status = -1;
    }

    ROS_INFO("Opened camera on %s", fd.c_str());
    loop_rate(fps);
  }

  void execute() {
    if(init_status != 0)
      return init_status;

    while(nh.ok())
    {
      cv::Mat frame;
      source >> frame;
      if (frame.empty())
        continue;

      // Convert to ROS image and send out
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      publisher.publish(msg);

      // Spin and wait
      cv::waitKey(1);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
  }
}

int main (int argc, char ** argv)
{
    ros::init(argc, argv, "image_publisher");

    ImagePublisher img_pub(ros::NodeHandle("~"));

    bool result;
    result = img_pub.execute();

    delete img_pub;

    return result;
}
