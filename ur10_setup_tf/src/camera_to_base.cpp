#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <stdio.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

// Intrinsic matrix
double cx = 323.2202;
double cy = 260.1301;
double fx = 523.6106;
double fy = 524.0606;
double depthScale = 1000.0;

//image
Mat color;
Mat depth;
static const std::string OPENCV_WINDOW = "Image window";

// Rotation and Translation
Eigen::Quaterniond q( -0.4789,   -0.5083,   -0.4938,    0.5181);
Eigen::Matrix3d rotation_matrix(q);
Eigen::Matrix3d R = rotation_matrix.transpose(); 
Eigen::Vector3d T (2.24016469144293,0.120338248925333,0.349860140274193);
Eigen::Vector3d point;
Eigen::Vector3d base_point;
ros::Publisher pose_pub;


void on_Mouse(int event, int x, int y, int flags, void *ustc)
{ 

  if (event == EVENT_LBUTTONDOWN)
  {
  //ROS_INFO_STREAM(R);
  //ROS_INFO_STREAM(T);
    geometry_msgs::Pose pose;
    if(depth.cols> 0 && depth.rows > 0)
      {
        unsigned int d = depth.ptr<unsigned short> (y)[x]; 
        if(d > 0)
        {
          point[2] = double(d)/depthScale; 
          point[0] = -(x-cx)*point[2]/fx;
          point[1] = -(y-cy)*point[2]/fy;  
          ROS_INFO_STREAM("x " << point[0] << "  y " << point[1] << "  z " << point[2]);
          base_point = R * point + T;
          pose.position.x = base_point[0];
          pose.position.y = base_point[1];
          pose.position.z = base_point[2];
          ROS_INFO_STREAM("base_point" << base_point);
          pose_pub.publish(pose);
        }
      }
  }
}

class ImageConverter
{

  image_transport::ImageTransport it_;
  image_transport::Subscriber rgb_image_sub_;
  image_transport::Subscriber depth_image_sub;



public:
  ImageConverter(ros::NodeHandle _nh): it_(_nh)
  {
    // Subscrive to input video feed and publish output video feed
    rgb_image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::rgbimageCb, this);
    depth_image_sub = it_.subscribe("/camera/depth/image_raw", 1,
      &ImageConverter::depthimageCb, this);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void rgbimageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      color = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window

  }

  void depthimageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
        depth = cv_ptr->image;
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // Update GUI Window

  }

    
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_to_base");
    ros::NodeHandle nh_;
    pose_pub = nh_.advertise<geometry_msgs::Pose>("pose", 1000);
    ImageConverter ic(nh_);
    setMouseCallback(OPENCV_WINDOW, on_Mouse,0); 
    
   // ros::MultiThreadedSpinner spinner(8); // Use 4 threads
   // spinner.spin(); // spin() will not return until the node has been shutdown

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::Duration(5).sleep();//wait
    while(ros::ok())
    {
    imshow(OPENCV_WINDOW, color);
    waitKey(3);
    imshow("depth", depth);
    waitKey(3);
    }
}
