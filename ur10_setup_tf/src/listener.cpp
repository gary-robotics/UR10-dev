#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n; 
  tf::TransformListener listener;
  tf::StampedTransform transform;
  geometry_msgs::Transform geo_transform;

  ros::Rate rate(1);
  rate.sleep();
  while(n.ok())
  {
      try
      {
        // from 2 to 1 
        listener.lookupTransform("base", "camera_link",ros::Time(0), transform);
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
      ROS_INFO_STREAM("X " << X);
      ROS_INFO_STREAM("Y " << Y);
      ROS_INFO_STREAM("Z " << Z);
      Eigen::Vector3d t(X,Y,Z);
      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(rotate,q);
      Eigen::Isometry3d T(q);
      T.pretranslate (t);
      Eigen::Matrix3d rotation_matrix;
      rotation_matrix =  q.toRotationMatrix();
      //ROS_INFO_STREAM(rotation_matrix);
      Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(0,1,2);
      //ROS_INFO_STREAM("euler_angles: " << euler_angles.transpose());   
      Eigen::AngleAxisd rotation_vector(q);
      ROS_INFO_STREAM("RX RY RZ: " << (rotation_vector.axis() * rotation_vector.angle()).transpose());
      //ROS_INFO_STREAM(rotation_vector.axis());
      //ROS_INFO_STREAM(rotation_vector.angle());
      rate.sleep();
  }


    return 0;
}
