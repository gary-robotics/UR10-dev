#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/Transform.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
using namespace std;
using namespace cv;


#define PI (3.1415926535897932346f)
static const std::string OPENCV_WINDOW = "Image window";
int i=0;

ofstream out("/home/ros/Desktop/cal/robot_data/base_to_end.txt");


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  tf::TransformListener listener;
  geometry_msgs::Transform geo_transform;
  tf::StampedTransform transform;


  char key;
  string image_name;



public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
      &ImageConverter::imageCb, this);    //original:/camera/rgb/image_raw
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    key = cv::waitKey(3);
    if(key =='s')
    {

      stringstream num;
      string x;
      num << i;
      num >> x;
      image_name = "/home/ros/Desktop/cal/Example Data/Images/image" + x + ".jpg";
      imwrite(image_name , cv_ptr->image);
      i++;

      try
      {
        listener.lookupTransform("base", "tool0_controller",ros::Time(0), transform);
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    
      tf::transformTFToMsg(transform, geo_transform);
      tf::Quaternion rotate = transform.getRotation();
      tfScalar X =  transform.getOrigin().getX();
      tfScalar Y = transform.getOrigin().getY();
      tfScalar Z = transform.getOrigin().getZ();
      ROS_INFO_STREAM("X " << X*1000);
      ROS_INFO_STREAM("Y " << Y*1000);
      ROS_INFO_STREAM("Z " << Z*1000);
      out << X*1000 <<" " << Y*1000 <<" " << Z*1000 <<" ";
      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(rotate,q);
      ROS_INFO_STREAM("w x y z: " << q.coeffs().w() <<" " << q.coeffs().x() << " " << q.coeffs().y() << " " << q.coeffs().z() <<endl);
      Eigen::Matrix3d rotation_matrix;
      rotation_matrix =  q.toRotationMatrix();
      Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(0,1,2);
      // ROS_INFO_STREAM("euler_angles: " << euler_angles.transpose());   
      Eigen::AngleAxisd rotation_vector(q);
      ROS_INFO_STREAM("RX RY RZ: " << (rotation_vector.axis() * rotation_vector.angle()).transpose());
      out <<  (rotation_vector.axis() * rotation_vector.angle()).transpose() << endl ;

    }
  }
};



int main(int argc, char** argv)
{

  ros::init(argc, argv, "base_to_tcp");
  ImageConverter ic;
  ros::spin();

}
