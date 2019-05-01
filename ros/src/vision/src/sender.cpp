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
  std::string fd;
  int fps;
  bool is_device;

  bool init_status;

public:
  /*
    @brief: fetch launch file params and initialize video capture.

    Using the node handle grab the required launch file parameters
    and create the ROS video topic to publish ROS images. After
    creation of the ROS topic, initialize the video capture with
    open cv selecting the device provided from getParam info.
   */
  ImagePublisher(ros::NodeHandle & nh):
    nh(nh), it(nh)
  {
    // Ger parameters from launch file.
    nh.getParam("topic_name", topic_name);
    nh.getParam("fd", fd);
    nh.getParam("fps", fps);
    nh.getParam("is_device", is_device);

    // Advertise video topic.
    it.advertise("/video/" + topic_name, 5);

    // TODO review device detection.
    if (is_device) {
      uint8_t video_index = (uint8_t) (fd.back() - '0');
      source = cv::VideoCapture(video_index);
    } else {
      source = cv::VideoCapture(fd);
    }


    // Validate that the cv::VideoCapture is opened.
    if (!source.isOpened()) {
      ROS_ERROR("Failed to open device on %s", fd.c_str());
      init_status = -1;
    }
    else
    {
      ROS_INFO("Opened camera on %s", fd.c_str());
      loop_rate(fps);
    }
  }

  /*
    @brief: if init success, start publishing ROS images.

    Publish the images to /video/<topic-name> ROS topic,
    at a frequency of ~fps~ Hz.
   */
  bool execute() {
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
    return init_status;
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
