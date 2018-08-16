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

bool success = false;
//bool start;

//class MoveitPlan
class PickObject
{

private:
    //moveit::planning_interface::MoveGroupInterface arm;
    //moveit::planning_interface::MoveGroupInterface gripper;
    moveit::planning_interface::PlanningSceneInterface plan_scene;
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
    robot_model_loader::RobotModelLoader robot_model_loader;
    moveit_msgs::Grasp grasp_candidate_;
    std::deque<std::pair<moveit_msgs::Grasp, ros::Time>> grasp_candidates_;
    string frame_id_;
    double grasp_offset_;
    std::mutex m_;
    ros::Subscriber sub_; 
    ros::Publisher pub_;

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
public: 
    moveit::planning_interface::MoveGroupInterface arm;
    moveit::planning_interface::MoveGroupInterface gripper;

    MoveitPlan(ros::NodeHandle &n): arm("arm"),gripper("gripper"),robot_model_loader("robot_description")
    {
          sub_ = n.subscribe("/detect_grasps/clustered_grasps",1000,&MoveitPlan::clustered_grasps_callback,this);

          // arm setttings
          arm.setMaxVelocityScalingFactor(0.01);
          arm.setPoseReferenceFrame("base");
          arm.setStartStateToCurrentState();
          arm.setPlannerId("RRTConnectkConfigDefault");
          arm.setWorkspace(-2,2,-2,2,-2,2);
          arm.setPlanningTime(5);
          arm.setGoalOrientationTolerance(0.04);
          arm.setGoalPositionTolerance(0.02);
       
          // Setting variables in grasp_candidate_ that are the same for every grasp
          grasp_offset_ = 0.02;
          grasp_candidate_.id = "grasp";
          
          grasp_candidate_.pre_grasp_approach.min_distance = 0.25;
          grasp_candidate_.pre_grasp_approach.desired_distance = 0.28;
               
          grasp_candidate_.post_grasp_retreat.min_distance = 0.13;
          grasp_candidate_.post_grasp_retreat.desired_distance = 0.15;
          grasp_candidate_.post_grasp_retreat.direction.header.frame_id = "base";
          grasp_candidate_.post_grasp_retreat.direction.vector.z = 1.0; 

          jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), ros::Duration(1.0), grasp_candidate_.pre_grasp_posture);
          jointValuesToJointTrajectory(gripper.getNamedTargetValues("close"), ros::Duration(2.0), grasp_candidate_.grasp_posture);

          // for signal
          //start = false;
    }

    /*bool hold()
    {
      arm.setNamedTarget("hold");
      arm.setStartStateToCurrentState();
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
      arm.setStartStateToCurrentState();
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
    }*/   

    bool pick()
    {
      ROS_INFO_STREAM("start MoveIt! Pick!");
      return success = arm.pick("",grasp_candidate_) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
      /*if(success)
        return 1;
      else 
        return 0;*/
    } 

  void clustered_grasps_callback(const gpd::GraspConfigList::ConstPtr& msg)
    {
      std::lock_guard<std::mutex> lock(m_);
      ros::Time grasp_stamp = msg->header.stamp;
      // camera_rgb_optical_frame
      frame_id_ = msg->header.frame_id;
      grasp_candidate_.grasp_pose.header.frame_id = frame_id_; 
      grasp_candidate_.pre_grasp_approach.direction.header.frame_id = frame_id_; 

      for(auto grasp:msg->grasps)
      {
        // shift the grasp according to the offset parameter
        grasp.top.x = grasp.top.x + grasp_offset_ * grasp.approach.x;
        grasp.top.y = grasp.top.y + grasp_offset_ * grasp.approach.y;
        grasp.top.z = grasp.top.z + grasp_offset_ * grasp.approach.z;

        grasp_candidate_.grasp_pose.pose = gpd_grasp_to_pose(grasp);

        grasp_candidate_.grasp_quality = grasp.score.data;
        grasp_candidate_.pre_grasp_approach.direction.vector.x = grasp.approach.x;
        grasp_candidate_.pre_grasp_approach.direction.vector.y = grasp.approach.y;
        grasp_candidate_.pre_grasp_approach.direction.vector.z = grasp.approach.z;

        //grasp_candidates_.push_front(std::make_pair(grasp_candidate_, grasp_stamp));
        //start = true;
      }
    }

    geometry_msgs::Pose gpd_grasp_to_pose(gpd::GraspConfig &grasp)
    {
      geometry_msgs::Pose pose;
      tf::Matrix3x3 orientation(grasp.approach.x, grasp.binormal.x, grasp.axis.x,
                                grasp.approach.y, grasp.binormal.y, grasp.axis.y,
                                grasp.approach.z, grasp.binormal.z, grasp.axis.z);

      tf::Quaternion orientation_quat;
      orientation.getRotation(orientation_quat);
      tf::quaternionTFToMsg(orientation_quat, pose.orientation);
      pose.position = grasp.top;
      return pose;        
    }


};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "moveit_plan");
  ros::NodeHandle n;

  //MoveitPlan *moveit_plan = new MoveitPlan(n);
  MoveitPlan MoveitPlan(n);
  PickObject Po;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ROS_INFO("Picking Object");

  while(ros::ok())
  {
    if(success)
    {
      Po.arm.setNamedTarget("place");
      while(!Po.arm.move())
        ROS_ERROR("moving to place pose failed.");
      Po.gripper.setNamedTarget("open");
      while(!Po.gripper.move())
        ROS_ERROR("opening gripper failed.");
      MP.arm.setNamedTarget("hold");
      while(!Po.arm.move())
        ROS_ERROR("moving hold failed.");
      ros::Duration(5).sleep();
      success = false;
    }
    success = Po.Pick();
  }

  return 0;





 /* while(ros::ok())
  {
    if(start)
    {
      if(moveit_plan->hold())
      {
        if(moveit_plan->pick())
        {
          if(moveit_plan->place())
          {
            ros::Duration(10).sleep();
            ROS_INFO_STREAM("Waiting ....");
            while(1);
          }
        }
      }
    }
  }
  return 0;*/
}
