#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <math.h>

// Rotation and Translation 
Eigen::Quaterniond camera_rotation( 0.009950416684597,0.710376335014308 , -0.703749771718084 , -0.001646410091099 ); // w x y z 
Eigen::Vector3d camera_translation(1.37996753865619, 0.219399064738647, 0.887545166846203);
Eigen::Isometry3d camera_to_base(camera_rotation);



int main(int argc, char *argv[])
{

  ros::init(argc, argv, "base_pose");
  ros::NodeHandle nh;
  ros::Publisher pose_pub;
  pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 1000);
  
  /*surface normal*/
  double x=0.167329, y=-0.3315, z=1.105;
  double n_x=0.0111406, n_y=-0.00943037, n_z=-0.999893;
  n_x = -(-n_x);
  n_y = -(-n_y);
  n_z = -n_z;
  Eigen::Matrix3d A,B,C;
  A << 1,0,0,
      0,n_z/sqrt(pow(n_y,2)+pow(n_z,2)),n_y/sqrt(pow(n_y,2)+pow(n_z,2)),
      0,-n_y/sqrt(pow(n_y,2)+pow(n_z,2)),n_z/sqrt(pow(n_y,2)+pow(n_z,2));
 // ROS_INFO_STREAM(A);
  B << sqrt(pow(n_y,2)+pow(n_z,2)) / sqrt(pow(n_x,2)+pow(n_y,2)+pow(n_z,2)),0,n_x/sqrt(pow(n_x,2)+pow(n_y,2)+pow(n_z,2)),
       0,1,0,
       -n_x/sqrt(pow(n_x,2)+pow(n_y,2)+pow(n_z,2)),0,sqrt(pow(n_y,2)+pow(n_z,2)) / sqrt(pow(n_x,2)+pow(n_y,2)+pow(n_z,2));
 // ROS_INFO_STREAM(B);
  C = (A * B).transpose();
  C = C.inverse().eval();
  Eigen::Quaterniond q(C);
 //ROS_INFO_STREAM(q.coeffs());
  
  camera_to_base.pretranslate(camera_translation);
  geometry_msgs::Pose camera_target;
  camera_target.position.x = -x;
  camera_target.position.y = -y;
  camera_target.position.z = z;
  camera_target.orientation.w =  q.coeffs().w();             
  camera_target.orientation.x =  q.coeffs().x();
  camera_target.orientation.y =  q.coeffs().y();
  camera_target.orientation.z =  q.coeffs().z();
  ROS_INFO_STREAM("camera_pose" << " " <<camera_target);
  
  tf::Pose camera_tf_target;
  tf::poseMsgToTF(camera_target,camera_tf_target);
  //ROS_INFO_STREAM(camera_to_base.matrix());
  Eigen::Affine3d camera_eigen_target;
  tf::poseTFToEigen(camera_tf_target,camera_eigen_target);

  Eigen::Affine3d eigen_target;
  tf::Pose tf_target;
  geometry_msgs::Pose target;
  eigen_target = camera_to_base * camera_eigen_target;
  tf::poseEigenToTF(eigen_target,tf_target);
  tf::poseTFToMsg(tf_target,target);

  ROS_INFO_STREAM( "base_pose" << " " <<target);




  while(ros::ok())
  {
      pose_pub.publish(target);
      ros::Duration(1).sleep();
  }


    return 0;
}