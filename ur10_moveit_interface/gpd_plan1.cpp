#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GraspPlanning.h>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <iostream>
#include <vector>
#include <string>
#include <mutex>


#include <gpd/GraspConfigList.h>
#include "grasp_interface/RCGripperCommand.h"

using namespace std;


//Eigen::Quaterniond camera_rotation (-0.00183452970917017, 0.0235215042208587, 0.999576722008115, 0.0170220492140404); // w x y z
//Eigen::Vector3d camera_translation(0.764110092170712, 0.569643355465629, 0.93149282964104);
Eigen::Quaterniond camera_rotation(0.117190437008716,-0.122919601861648,0.732033580840784,-0.659760569807942); // HK_kinect w x y z 
Eigen::Vector3d camera_translation(0.278292947573651,1.32699493388865,0.26265011950633);  // HK_kinect w x y z
Eigen::Isometry3d camera_to_base(camera_rotation);

bool start;

class MoveitPlan
{

private:
    moveit::planning_interface::MoveGroupInterface arm;
    moveit::planning_interface::MoveGroupInterface gripper;
    moveit::planning_interface::PlanningSceneInterface plan_scene;
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    const robot_state::JointModelGroup * joint_model_group;
    moveit_msgs::Grasp grasp_candidate_;
    string frame_id_;
    std::vector<double> joint_values;
    double grasp_offset_;
    std::mutex m_;
    ros::Subscriber sub_; 
    ros::Publisher pub_;
    geometry_msgs::Pose grasp_pose;
    geometry_msgs::Pose pre_grasp_pose;
public: 
    MoveitPlan(ros::NodeHandle &n): arm("arm"),gripper("gripper"),robot_model_loader("robot_description")
    {
          ROS_INFO_STREAM("init");
          sub_ = n.subscribe("/detect_grasps/clustered_grasps",1000,&MoveitPlan::clustered_grasps_callback,this);
          pub_ = n.advertise<grasp_interface::RCGripperCommand>("grip_command",1);

       
          arm.setMaxVelocityScalingFactor(0.04);
          arm.setPoseReferenceFrame("base");
          arm.setStartStateToCurrentState();
          arm.setPlannerId("RRTConnectkConfigDefault");
          arm.setWorkspace(-2,2,-2,2,-2,2);
          arm.setPlanningTime(20);
          arm.setGoalOrientationTolerance(0.03);
          arm.setGoalPositionTolerance(0.01);

          grasp_interface::RCGripperCommand gp;
          
          // Setting variables in grasp_candidate_ that are the same for every grasp
          grasp_offset_ = 0;
          grasp_candidate_.id = "grasp";
          
          grasp_candidate_.pre_grasp_approach.min_distance = 0.08;
          grasp_candidate_.pre_grasp_approach.desired_distance = 0.10;
               
          grasp_candidate_.post_grasp_retreat.min_distance = 0.13;
          grasp_candidate_.post_grasp_retreat.desired_distance = 0.15;
          grasp_candidate_.post_grasp_retreat.direction.header.frame_id = arm.getPlanningFrame();
          grasp_candidate_.post_grasp_retreat.direction.vector.z = 1.0; 
          
          jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), ros::Duration(1.0), grasp_candidate_.pre_grasp_posture);
          jointValuesToJointTrajectory(gripper.getNamedTargetValues("close"), ros::Duration(2.0), grasp_candidate_.grasp_posture);
          
          kinematic_model = robot_model_loader.getModel();
          joint_model_group = kinematic_model->getJointModelGroup("arm");
          start = false;
    }

    bool pre_grasp()
    {
      if(start)
      {
        robot_state::RobotState kinematic_state(*arm.getCurrentState());
        bool found_ik = kinematic_state.setFromIK(joint_model_group,pre_grasp_pose);
        if(found_ik)
        {
          ROS_INFO_STREAM("succeed to find IK for pre-grasp");
          gripper.setNamedTarget("open");
          gripper.move();
          gripper.setNamedTarget("open");
          gripper.move();
          gripper.setNamedTarget("open");
          gripper.move();
          arm.setPoseTarget(pre_grasp_pose);
          bool success = arm.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
          if(success)
          {
            ROS_INFO_STREAM("succeed to plan for pre-grasp");
            ROS_INFO_STREAM(pre_grasp_pose);

            arm.move();
            return 1;
          }
          else
          {
            ROS_INFO_STREAM("fail to plan for pre-grasp");
            return 0;
          }

        } 
        else
        {
          ROS_INFO_STREAM("fail to find IK for pre-grasp");
          return 0;
        }

      }
    }

    bool grasp()
    {
      if(start)
      {
        robot_state::RobotState kinematic_state(*arm.getCurrentState());
        bool found_ik = kinematic_state.setFromIK(joint_model_group,grasp_pose);
        if(found_ik)
        {
          ROS_INFO_STREAM("succeed to find IK for grasp");
          arm.setPoseTarget(grasp_pose);
          bool success = arm.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
          if(success)
          {
            ROS_INFO_STREAM("succeed to plan for grasp");
            ROS_INFO_STREAM(grasp_pose);
            arm.move();
            ros::Duration(1).sleep();
            gripper.setNamedTarget("close");
            gripper.move();
            gripper.setNamedTarget("close");
            gripper.move();
            gripper.setNamedTarget("close");
            gripper.move();
            return 1;
          }
          else
          {
            ROS_INFO_STREAM("fail to plan for grasp");
            return 0;
          }

        }
        else
        {
          ROS_INFO_STREAM("fail to find IK for grasp");
          return 0;
        }
      }
    }

    bool hold()
    {

      arm.setNamedTarget("hold");
      bool success = arm.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
      if(success)
      {
        ROS_INFO_STREAM("succeed to plan for hold");
        arm.move();
        return 1;
      }
      else
      {
        ROS_INFO_STREAM("fail to plan for hold");
        return 0;
      }
    }

    bool place()
    {
      arm.setNamedTarget("place");
      bool success = arm.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
      if(success)
      {
        ROS_INFO_STREAM("succeed to plan for place");
        arm.move();
        gripper.setNamedTarget("open");
        gripper.move();
        return 1;
      }
      else
      {
        ROS_INFO_STREAM("succeed to plan for place");
        return 0;
      }      
    }
    
    void stop()
    {     
      gripper.setNamedTarget("close");
      gripper.move();
      gripper.setNamedTarget("close");
      gripper.move();
      arm.stop();
    }

    void jointValuesToJointTrajectory(std::map<std::string, double> target_values, ros::Duration duration,
            trajectory_msgs::JointTrajectory &grasp_pose)
    {
        grasp_pose.joint_names.reserve(target_values.size());
        grasp_pose.points.resize(1);
        grasp_pose.points[0].positions.reserve(target_values.size());

        for(std::map<std::string, double>::iterator it = target_values.begin(); it != target_values.end(); ++it)
        {
          grasp_pose.joint_names.push_back(it->first);
          grasp_pose.points[0].positions.push_back(it->second);
        }
        grasp_pose.points[0].time_from_start = duration;
    }

  void clustered_grasps_callback(const gpd::GraspConfigList::ConstPtr& msg)
    {
      ROS_INFO_STREAM("cb");
      std::lock_guard<std::mutex> lock(m_);
      ros::Time grasp_stamp = msg->header.stamp;
      frame_id_ = msg->header.frame_id;
      grasp_candidate_.grasp_pose.header.frame_id = frame_id_;
      grasp_candidate_.pre_grasp_approach.direction.header.frame_id = frame_id_;

      for(auto grasp:msg->grasps)
      {
        // shift the grasp according to the offset parameter
        grasp = msg->grasps[0];
        grasp.top.x = grasp.top.x + grasp_offset_ * grasp.approach.x;
        grasp.top.y = grasp.top.y + grasp_offset_ * grasp.approach.y;
        grasp.top.z = grasp.top.z + grasp_offset_ * grasp.approach.z;


        grasp_candidate_.grasp_pose.pose = gpd_grasp_to_pose(grasp);

        grasp_candidate_.grasp_quality = grasp.score.data;
        grasp_candidate_.pre_grasp_approach.direction.vector.x = grasp.approach.x;
        grasp_candidate_.pre_grasp_approach.direction.vector.y = grasp.approach.y;
        grasp_candidate_.pre_grasp_approach.direction.vector.z = grasp.approach.z;
        
        start = true;
      }
    }

    geometry_msgs::Pose gpd_grasp_to_pose(gpd::GraspConfig &grasp)
    {
        

        tf::Matrix3x3 orientation(grasp.approach.x, grasp.binormal.x, grasp.axis.x,
                                  grasp.approach.y, grasp.binormal.y, grasp.axis.y,
                                  grasp.approach.z, grasp.binormal.z, grasp.axis.z);
        Eigen::Matrix3d rotation_matrix;
        tf::matrixTFToEigen(orientation,rotation_matrix);

        Eigen::Vector4d g(grasp.approach.x,grasp.approach.y,grasp.approach.z,1);
        Eigen::Vector4d b;        
        b = camera_to_base * g;

        Eigen::Quaterniond q1(rotation_matrix);
        Eigen::Isometry3d T_Grasp(q1);
        T_Grasp.pretranslate(Eigen::Vector3d(grasp.top.x,grasp.top.y,grasp.top.z));
        Eigen::Isometry3d T_Base;
        T_Base = camera_to_base * T_Grasp;
        Eigen::Matrix3d base_r = T_Base.rotation();
        Eigen::Vector3d base_t = T_Base.translation();
        tf::Quaternion q;
        tf::Vector3 t;
        Eigen::Quaterniond q2(base_r);
        tf::quaternionEigenToTF(q2,q);
        tf::vectorEigenToTF(base_t,t);

        grasp_pose.position.x = t.getX();
        grasp_pose.position.y = t.getY();
        grasp_pose.position.z = t.getZ() + 0.05;
        grasp_pose.orientation.x = q.getX();
        grasp_pose.orientation.y = q.getY();
        grasp_pose.orientation.z = q.getZ();
        grasp_pose.orientation.w = q.getW();

        pre_grasp_pose = grasp_pose;
        pre_grasp_pose.position.x = pre_grasp_pose.position.x - pre_grasp_pose.position.x * b[0];
        pre_grasp_pose.position.y = pre_grasp_pose.position.y - pre_grasp_pose.position.y * b[1];
        pre_grasp_pose.position.z = pre_grasp_pose.position.z - pre_grasp_pose.position.z * b[2];

      
      
        /*
        tf::Quaternion orientation_quat;
        orientation.getRotation(orientation_quat);
        tf::quaternionTFToMsg(orientation_quat, pose.orientation);
        pose.position = grasp.top;
        */

        return grasp_pose;
    }
};




int main(int argc, char **argv)
{

  ros::init(argc, argv, "moveit_plan");
  ros::NodeHandle n;
  camera_to_base.pretranslate(camera_translation);
  MoveitPlan *moveit_plan = new MoveitPlan(n);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ROS_INFO_STREAM("in main");
  while(ros::ok())
  {
    if(moveit_plan->hold())
    {
      if(start)
      {
        {
          ROS_INFO_STREAM("arrive at hold pose");
          if(moveit_plan->pre_grasp())
          {
            ROS_INFO_STREAM("arrive at pre-grasp pose");
            if(moveit_plan->grasp())
            {
              ROS_INFO_STREAM("arrive at grasp pose");
              if(moveit_plan->place())
              {
                ROS_INFO_STREAM("arrive at place pose");
                continue;
              }
            }
          }
        }
      }
    }
  }

  moveit_plan->stop();
  return 0;
}
