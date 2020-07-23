#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

int _index = 0;

std::ofstream file;

static string toString(const Eigen::VectorXd& mat)
{
    std::stringstream ss;
    ss << mat.transpose();
    return ss.str();
}

void callback(const ImageConstPtr& rgb_img, const ImageConstPtr& depth_img)
{
  try
  {
    std_msgs::Header h= rgb_img->header;
    
    std::stringstream ss;
    ss << h.stamp;
    file << ss.str() <<std::endl;

    cv_bridge::CvImagePtr cv_ptr_rgb, cv_ptr_depth;
    cv_ptr_rgb = cv_bridge::toCvCopy(rgb_img, sensor_msgs::image_encodings::BGR8);
    cv_ptr_depth = cv_bridge::toCvCopy(depth_img);
    std::string f_rgb = "rgb/r" + std::to_string(_index)+".png";
    std::string f_depth = "depth/d" + std::to_string(_index)+".png";
    cv::imwrite(f_rgb,cv_ptr_rgb->image);
    cv::imwrite(f_depth,cv_ptr_depth->image);
    
    _index = _index + 1;
    ROS_INFO("Received data %i", _index);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
}

int main(int argc, char **argv)
{
  ROS_INFO("RGBD - data collecting node running!\nWaiting for topics to publish!");
  ros::init(argc, argv, "rgbd_dc");
  ros::NodeHandle nh;

  file.open("timestamps.txt");
 
  // message_filters::Subscriber<Image> rgb_sub(nh, "/kinect2/hd/image_color_rect", 1);
  // message_filters::Subscriber<Image> depth_sub(nh, "/kinect2/hd/image_depth_rect", 1); 
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(20), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  ros::spin();
  file.close();
}


