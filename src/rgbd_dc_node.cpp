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
#include <sensor_msgs/Imu.h>
#include <string>
#include <message_filters/sync_policies/approximate_time.h>
#include <eigen3/Eigen/Core>

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

void callback2(const sensor_msgs::ImuConstPtr& imu_accel, const sensor_msgs::ImuConstPtr& imu_gyro)
{
  try
  {
    std_msgs::Header h= imu_accel->header;
    Eigen::Vector3d imu_w = imu_w; 
    imu_w << imu_gyro->angular_velocity.x, imu_gyro->angular_velocity.y, imu_gyro->angular_velocity.z;
    Eigen::Vector3d imu_a; 
    imu_a << imu_accel->linear_acceleration.x, imu_accel->linear_acceleration.y, imu_accel->linear_acceleration.z;
    std::stringstream ss;
    ss << h.stamp;
    file << ss.str() <<" "<< toString(imu_a) <<" "<< toString(imu_w) <<std::endl;
    ROS_INFO("Received data %s", ss.str());
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
}

void callback(const ImageConstPtr& rgb_img, const ImageConstPtr& depth_img, const sensor_msgs::ImuConstPtr& imu_accel, const sensor_msgs::ImuConstPtr& imu_gyro)
{
  try
  {
    std_msgs::Header h= rgb_img->header;
    
    Eigen::Vector3d imu_w = imu_w; 
    imu_w << imu_gyro->angular_velocity.x, imu_gyro->angular_velocity.y, imu_gyro->angular_velocity.z;
    Eigen::Vector3d imu_a; 
    imu_a << imu_accel->linear_acceleration.x, imu_accel->linear_acceleration.y, imu_accel->linear_acceleration.z;
    ROS_INFO("imu_a: [%d %d %d] imu_w: [%d %d %d] \n",imu_a[0], imu_a[1], imu_a[2] , imu_w[0], imu_w[1], imu_w[2]);
    std::stringstream ss;
    ss << h.stamp;
    file << ss.str() << toString(imu_a) << toString(imu_w) <<std::endl;

    cv_bridge::CvImagePtr cv_ptr_rgb, cv_ptr_depth;
    cv_ptr_rgb = cv_bridge::toCvCopy(rgb_img, sensor_msgs::image_encodings::BGR8);
    cv_ptr_depth = cv_bridge::toCvCopy(depth_img);
    std::string f_rgb = "rgb/r" + std::to_string(_index)+".png";
    std::string f_depth = "depth/d" + std::to_string(_index)+".png";
    cv::imwrite(f_rgb,cv_ptr_rgb->image);
    cv::imwrite(f_depth,cv_ptr_depth->image);
    
    _index = _index + 1;
    //ROS_INFO("Received data %i", _index);

  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
}

int main(int argc, char **argv)
{
  ROS_INFO("rgbd + imu - data collecting node running!\nWaiting for topics to publish!");
  ros::init(argc, argv, "rgbd_dc");
  ros::NodeHandle nh;

  file.open("timestamps_IMU_raw.txt");
 
  // message_filters::Subscriber<Image> rgb_sub(nh, "/kinect2/hd/image_color_rect", 1);
  // message_filters::Subscriber<Image> depth_sub(nh, "/kinect2/hd/image_depth_rect", 1); 
  // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
  // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Imu> imu_accel(nh, "/camera/accel/sample", 1);
  message_filters::Subscriber<sensor_msgs::Imu> imu_gyro(nh, "/camera/gyro/sample", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(20), imu_accel, imu_gyro);
  sync.registerCallback(boost::bind(&callback2, _1, _2));
  
  //message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect_raw", 1);
  //ROS_INFO("topics: /camera/color/image_raw /camera/depth/image_rect_raw");
  
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu, sensor_msgs::Imu> sync_pol;
  // message_filters::Synchronizer<sync_pol> sync(sync_pol(20), rgb_sub, depth_sub, imu_accel, imu_gyro);
  // sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  
  // Old version I used TimeSynchronizer -- sometimes didn't work.
  // TimeSynchronizer<Image, Image> sync(rgb_sub, depth_sub, 10);
  // sync.registerCallback(boost::bind(&callback, _1, _2));
  
  ros::spin();
  file.close();
}

/* - Only one topic!
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    std::string filename = "r" + std::to_string(_index)+".png";
    cv::imwrite(filename,cv_ptr->image);
    _index = _index + 1;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rgbd_dc");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/kinect2/hd/image_color", 1, &imageCallback);
  ros::spin();
}
*/



