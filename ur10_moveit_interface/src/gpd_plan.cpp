#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GraspPlanning.h>

#include <iostream>
#include <vector>
#include <string>
#include <mutex>

#include <gpd/GraspConfigList.h>
#include "grasp_interface/RCGripperCommand.h"

using namespace std;

/*
Eigen::Quaterniond camera_rotation (-0.00183452970917017, 0.0235215042208587, 0.999576722008115, 0.0170220492140404); // w x y z
Eigen::Vector3d camera_translation(0.764110092170712, 0.569643355465629, 0.93149282964104);

Eigen::Quaterniond camera_rotation(0.117190437008716,-0.122919601861648,0.732033580840784,-0.659760569807942); // HK_kinect w x y z 
Eigen::Vector3d camera_translation(0.278292947573651,1.32699493388865,0.26265011950633);  // HK_kinect w x y z
*/

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
    moveit_msgs::Grasp grasp_candidate_;
    vector<moveit_msgs::Grasp> grasp_candidates_;
    string frame_id_;
    double grasp_offset_;
    std::mutex m_;
    ros::Subscriber sub_; 
    ros::Publisher pub_;
    bool success;
    //*********place*****************//
    std::vector<moveit_msgs::PlaceLocation> place_location;


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
    MoveitPlan(ros::NodeHandle &n): arm("arm"),gripper("gripper"),robot_model_loader("robot_description")
    {
          sub_ = n.subscribe("/detect_grasps/clustered_grasps",1000,&MoveitPlan::clustered_grasps_callback,this);
          // arm setttings
          arm.setPoseReferenceFrame("base");
          arm.setStartStateToCurrentState();
          arm.setPlannerId("RRTConnectkConfigDefault");
          arm.setWorkspace(-2,2,-2,2,-2,2);
          arm.setPlanningTime(5);
          arm.setGoalOrientationTolerance(0.03);
          arm.setGoalPositionTolerance(0.01);
       
          // Setting variables in grasp_candidate_ that are the same for every grasp
          grasp_offset_ = 0.02;
          grasp_candidate_.id = "grasp";
          
          grasp_candidate_.pre_grasp_approach.min_distance = 0.20;
          grasp_candidate_.pre_grasp_approach.desired_distance = 0.25;
               
          grasp_candidate_.post_grasp_retreat.min_distance = 0.20;
          grasp_candidate_.post_grasp_retreat.desired_distance = 0.25;
          grasp_candidate_.post_grasp_retreat.direction.header.frame_id = "base";
          grasp_candidate_.post_grasp_retreat.direction.vector.z = 1.0; 


          //*************setting place location pose*******************//
          place_location.resize(1);
          place_location[0].place_pose.header.frame_id = "base";
          place_location[0].place_pose.pose.orientation.x = 0.828444;
          place_location[0].place_pose.pose.orientation.y = -0.555514;
          place_location[0].place_pose.pose.orientation.z = 0.0243543;
          place_location[0].place_pose.pose.orientation.w = 0.0670162;
          place_location[0].place_pose.pose.position.x = 0.705;
          place_location[0].place_pose.pose.position.y = -0.175;
          place_location[0].place_pose.pose.position.z = 0.630;

          place_location[0].pre_place_approach.direction.header.frame_id = "base";

          place_location[0].pre_place_approach.direction.vector.z = -1.0;
          place_location[0].pre_place_approach.min_distance = 0.095;
          place_location[0].pre_place_approach.desired_distance = 0.115;
          
          place_location[0].post_place_retreat.direction.header.frame_id = "base";
          /* Direction is set as negative y axis */
          place_location[0].post_place_retreat.direction.vector.y = -1.0;
          place_location[0].post_place_retreat.min_distance = 0.1;
          place_location[0].post_place_retreat.desired_distance = 0.25;

          jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), ros::Duration(1.0), grasp_candidate_.pre_grasp_posture);
          jointValuesToJointTrajectory(gripper.getNamedTargetValues("close"), ros::Duration(2.0), grasp_candidate_.grasp_posture);
          jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), ros::Duration(1.0), place_location[0].post_place_posture);

          // for signal

    }

    bool hold()
    {
      arm.setNamedTarget("hold");
      success = arm.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
      if(success)
      {
        ROS_INFO_STREAM("succeed to plan for hold");
        arm.move(); 
        ros::Duration(0.5).sleep();
        gripper.setNamedTarget("open");
        gripper.move();       
        return 1;
      }
      else
      {
        ROS_ERROR("fail to plan for hold");
        return 0;
      }
    }

    void clear()
    {
      ROS_INFO_STREAM("clear pose from GPD...");
      vector<moveit_msgs::Grasp>().swap(grasp_candidates_);
    } 
    


    /*
    bool place()
    {
      arm.setNamedTarget("place");
      success = arm.plan(arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
      if(success)
      {
        ROS_INFO_STREAM("succeed to plan for place");
        arm.move();
        ros::Duration(0.5).sleep();
        gripper.setNamedTarget("open");
        gripper.move();
        gripper.setNamedTarget("open");
        gripper.move();
        gripper.setNamedTarget("open");
        gripper.move();
        return 1;
      }
      else
      {
        ROS_INFO_STREAM("fail to plan for place");
        return 0;
      }      
    }
    */  

    bool pick()
    {
      ROS_INFO_STREAM("start Pick!");
      success = arm.pick("",grasp_candidates_) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
      ROS_INFO_STREAM("Pick succeed or not? " << success);
      return !success;
    } 

    bool place()
    {
      ROS_INFO_STREAM("start Place!");
      success = arm.place("",place_location) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
      ROS_INFO_STREAM("Place succeed or not? " << success);
      return success;
    }    

  void clustered_grasps_callback(const gpd::GraspConfigList::ConstPtr& msg)
    {
      std::lock_guard<std::mutex> lock(m_);
      ros::Time grasp_stamp = msg->header.stamp;
      frame_id_ = msg->header.frame_id;   // camera_rgb_optical_frame
      ROS_INFO_STREAM(frame_id_<< endl);
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

        grasp_candidates_.push_back(grasp_candidate_);
        start = true;
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

  MoveitPlan mp(n);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Picking Object");

  while(ros::ok())
  {
    if(start)
    {
      if(mp.hold())
      {
        ros::Duration(3).sleep();
        mp.pick();
        {
          ros::Duration(1).sleep();
          mp.clear();
          start = false;
        }
      }
    }
  }
  return 0;
}
