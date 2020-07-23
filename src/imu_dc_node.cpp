#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <message_filters/sync_policies/approximate_time.h>
#include <eigen3/Eigen/Core>

using namespace std;
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

void callback(const sensor_msgs::ImuConstPtr& imu_accel, const sensor_msgs::ImuConstPtr& imu_gyro)
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

int main(int argc, char **argv)
{
  ROS_INFO("IMU - data collecting node running!\nWaiting for topics to publish!");
  ros::init(argc, argv, "imu_dc");
  ros::NodeHandle nh;

  file.open("imu_raw_data.txt");
  file << "# Data Format: timestamp a_x a_y a_z w_x w_y w_z"<< std::endl;
 
  message_filters::Subscriber<sensor_msgs::Imu> imu_accel(nh, "/camera/accel/sample", 1);
  message_filters::Subscriber<sensor_msgs::Imu> imu_gyro(nh, "/camera/gyro/sample", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(20), imu_accel, imu_gyro);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  ros::spin();
  file.close();
}




