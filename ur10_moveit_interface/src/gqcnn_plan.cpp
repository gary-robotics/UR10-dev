#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "grasp_interface/RCGripperCommand.h"

#include <iostream>
#include <vector>
using namespace std;


geometry_msgs::Pose target_pose;
geometry_msgs::Pose pre_grasp_pose;
int start = 0;

/* hand_eye_calibration result */
Eigen::Quaterniond camera_rotation (0.022605849585286,   0.710758257317346,   0.702988984346954,   0.010870285487253); // w x y z 
Eigen::Vector3d camera_translation(1.37996753865619, 0.219399064738647, 0.887545166846203);
Eigen::Isometry3d camera_to_base(camera_rotation);



void poseCallback(const geometry_msgs::Pose msg)
{

  ROS_INFO_STREAM("in cb");
  tf::Pose p;
  Eigen::Affine3d e;
  tf::poseMsgToTF(msg,p);
  tf::poseTFToEigen(p,e);
  Eigen::Affine3d tp;
  tp = camera_to_base * e ;
  tf::Pose p_;
  tf::poseEigenToTF(tp,p_);
  tf::poseTFToMsg(p_,target_pose);
  pre_grasp_pose = target_pose;
  pre_grasp_pose.position.z += 0.40;
  target_pose.position.z += 0.16;
  ROS_INFO_STREAM(target_pose);
  start = 1;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "move_plan");
  ros::NodeHandle nh;
  tf::TransformListener listener;
  ros::Duration(1).sleep();
  ros::Subscriber sub = nh.subscribe("gqcnn_grasp_pose",1,poseCallback);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Duration(2).sleep();
  /* set transform */
  camera_to_base.pretranslate(camera_translation);

  /* move group setting */
  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface gripper("gripper");
  ROS_INFO("Reference frame: %s", arm.getPlanningFrame().c_str());  
  ROS_INFO("Reference frame: %s", arm.getEndEffectorLink().c_str());
  arm.setStartStateToCurrentState();
  arm.setPlannerId("RRTConnectkConfigDefault");
  arm.setPlanningTime(10);
  arm.setMaxVelocityScalingFactor(0.02);
  arm.setPoseReferenceFrame("base");
  arm.allowReplanning(true);
  arm.setWorkspace(-2,2,-2,2,-2,2);
  gripper.setNamedTarget("open");
  gripper.move();

  /*
  arm.setNamedTarget("hold");
  arm.move();
  */
  
  /* gripper interface */
  /*
  ros::Publisher pub_= nh.advertise<grasp_interface::RCGripperCommand>("grip_command",1);
  grasp_interface::RCGripperCommand gp;
  */

  /* activate the gripper 255=close 0=open*/
  /*
  gp.position = 255;
  pub_.publish(gp);
  ros::Duration(2).sleep();
  gp.position = 0;
  pub_.publish(gp);
  ros::Duration(2).sleep();
  */

  ROS_INFO_STREAM("in main");

  while(ros::ok() )
 {
   if(start == 1)
   {
     
    /* add visualization*/
    tf::Quaternion tf_rotation;
    tf::quaternionMsgToTF(target_pose.orientation, tf_rotation);
    Eigen::Quaterniond eigen_rotation;
    tf::quaternionTFToEigen(tf_rotation, eigen_rotation);
    Eigen::AngleAxisd rotation_vector(eigen_rotation);
    ROS_INFO_STREAM("RX RY RZ: " << (rotation_vector.axis() * rotation_vector.angle()).transpose());

    tf::Pose tf_pose;
    Eigen::Affine3d end_effector_state;

    robot_state::RobotState kinematic_state(*arm.getCurrentState());
    
    const robot_state::JointModelGroup *joint_model_arm = arm.getCurrentState()->getJointModelGroup("arm");;
    tf::poseMsgToTF(target_pose,tf_pose);
    tf::poseTFToEigen(tf_pose, end_effector_state);
    bool found_ik = kinematic_state.setFromIK(joint_model_arm, target_pose); 
    ROS_INFO_STREAM("found_ik" << " " << found_ik);
    if(found_ik)
    {
      std::vector<double> joint_values;
      kinematic_state.copyJointGroupPositions(joint_model_arm, joint_values);
    }
    
    arm.setPoseTarget(end_effector_state);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if(success)
    {
      ROS_INFO_STREAM("plan succeed");
      /* first to pre-grasp pose*/
      arm.setPoseTarget(pre_grasp_pose);
      arm.move();
      ros::Duration(2).sleep();
      /* grasp pose */
      arm.execute(plan);
      ros::Duration(5).sleep();
      arm.clearPoseTarget();
      /* gripper grasp */
      gripper.setNamedTarget("close");
      gripper.move();
      ros::Duration(5).sleep();
      /* move to pre-difined place pose*/
      arm.setNamedTarget("hold");
      arm.move();
      ros::Duration(5).sleep();
      /* gripper open */
      gripper.setNamedTarget("open");
      ros::Duration(5).sleep();  

      /*
      gp.position = 255;
      gp.force = 50;
      pub_.publish(gp);
 
      arm.setNamedTarget("hold");
      arm.move();
      gp.position = 0;
      pub_.publish(gp);
      ros::Duration(2).sleep();
      */

      start = 0;
    }
    
    
   }
   
 }

 arm.stop();
 

}
