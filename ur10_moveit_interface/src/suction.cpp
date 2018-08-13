#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <moveit/move_group_interface/move_group_interface.h>

#include <math.h>

// Rotation and Translation 
Eigen::Quaterniond camera_rotation (0.011651763924937,0.724653150487702,0.689015272988343,-0.000038399023251); // w x y z
Eigen::Vector3d camera_translation(1.236198544416497, 0.243724848741023, 0.958845055486651);
Eigen::Isometry3d camera_to_base(camera_rotation);



int main(int argc, char *argv[])
{

  ros::init(argc, argv, "base_pose");
  ros::NodeHandle nh;
  ros::Publisher pose_pub;
  pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 1000);
  

  
  /* surface normal calculation, needs to be changed to cb function */
  double x=0.118171, y=-0.0785094, z=0.380545;
  double n_x=-0.468805, n_y=0.132566, n_z=-0.873297;
  n_x = -n_x;
  n_y = -n_y;
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
  camera_target.position.x = x;
  camera_target.position.y = y;
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



  /* Moveit! setup */
  moveit::planning_interface::MoveGroupInterface arm("arm");
  arm.setStartStateToCurrentState();
  arm.setPoseReferenceFrame("base");
  arm.setPlannerId("LBKPIECEkConfigDefault");
  arm.setPlanningTime(10);
  arm.setMaxVelocityScalingFactor(0.02);
  arm.setWorkspace(-2,2,-2,2,-2,2);
  ros::Duration(1).sleep();

  robot_state::RobotState kinematic_state(*arm.getCurrentState());
  const robot_state::JointModelGroup *joint_model_group = arm.getCurrentState()->getJointModelGroup("arm");
  bool founk_ik = kinematic_state.setFromIK(joint_model_group,target);
  if(founk_ik)
  {
    ROS_INFO_STREAM("succeed to find IK");
    arm.setPoseTarget(target);
    arm.move();
  }
  else
  {
    ROS_INFO_STREAM("cannot find IK for this goal pose");
  }


    return 0;
}