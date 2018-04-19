
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


Eigen::Quaterniond camera_rotation (0.022605849585286,   0.710758257317346,   0.702988984346954,   0.010870285487253); // w x y z
Eigen::Vector3d camera_translation(1.37996753865619, 0.219399064738647, 0.887545166846203);
Eigen::Isometry3d camera_to_base(camera_rotation);



class MoveitPlan
{

private:
    moveit::planning_interface::MoveGroupInterface arm;
    moveit::planning_interface::MoveGroupInterface gripper;
    moveit::planning_interface::PlanningSceneInterface plan_scene;
    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model ;
    moveit_msgs::Grasp grasp_candidate_;
    string frame_id_;
    std::vector<double> joint_values;
    const robot_state::JointModelGroup * joint_model_group;
    bool start;
    double grasp_offset_;
    std::mutex m_;
    ros::Subscriber sub_; 
    ros::Publisher pub_;
    geometry_msgs::Pose pose;

public: 
    MoveitPlan(ros::NodeHandle &n): arm("arm"),gripper("gripper"),robot_model_loader("robot_description")
    {
          ROS_INFO_STREAM("init");
          sub_ = n.subscribe("/detect_grasps/clustered_grasps",1000,&MoveitPlan::clustered_grasps_callback,this);
          pub_ = n.advertise<grasp_interface::RCGripperCommand>("grip_command",1);
          grasp_interface::RCGripperCommand gp;
       
          arm.setMaxVelocityScalingFactor(0.02);
          arm.setPoseReferenceFrame("base");
          arm.allowReplanning(true);
          arm.setNumPlanningAttempts(5);
          arm.setStartStateToCurrentState();
          arm.setPlannerId("RRTConnectkConfigDefault");
          arm.setWorkspace(-2,2,-2,2,-2,2);
          arm.setPlanningTime(20);


          // Setting variables in grasp_candidate_ that are the same for every grasp
          grasp_offset_ = 0;
          grasp_candidate_.id = "grasp";

          grasp_candidate_.pre_grasp_approach.min_distance = 0.08;
          grasp_candidate_.pre_grasp_approach.desired_distance = 0.1;
          //grasp_candidate_.pre_grasp_approach.direction.header.frame_id = "tool0";

/*
          grasp_candidate_.post_grasp_retreat.min_distance = 0.13;
          grasp_candidate_.post_grasp_retreat.desired_distance = 0.15;
          grasp_candidate_.post_grasp_retreat.direction.header.frame_id = arm.getPlanningFrame();
          grasp_candidate_.post_grasp_retreat.direction.vector.z = 1.0; 
*/


          jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), ros::Duration(1.0), grasp_candidate_.pre_grasp_posture);
          jointValuesToJointTrajectory(gripper.getNamedTargetValues("close"), ros::Duration(2.0), grasp_candidate_.grasp_posture);
          
          kinematic_model = robot_model_loader.getModel();
          ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
          joint_model_group = kinematic_model->getJointModelGroup("arm");
          start = false;
    }

    bool plan()
    {
      if(start)
      {
        arm.setPoseTarget(pose);
       // arm.pick("",grasp_candidate_);
        return 1;
      }
      else
      return 0;
    }


    bool move()
    {
    if(this->arm.move())
    {
        ros::Duration(5).sleep();
        this->arm.clearPoseTargets ();
        return 1;
    }
    else
        return 0;
    }

    bool stop()
    {
      this->arm.stop();
      return 1;
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
      frame_id_ = "camera_link";
      ROS_INFO_STREAM(frame_id_);
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

        pose.position.x = t.getX();
        pose.position.y = t.getY();
        pose.position.z = t.getZ();
        pose.orientation.x = q.getX();
        pose.orientation.y = q.getY();
        pose.orientation.z = q.getZ();
        pose.orientation.w = q.getW();
      
        
        /*
        tf::Quaternion orientation_quat;
        orientation.getRotation(orientation_quat);
        tf::quaternionTFToMsg(orientation_quat, pose.orientation);

        pose.position = grasp.top;
        */
        
        ROS_INFO_STREAM(pose);

        return pose;
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
  if(moveit_plan->plan())
    if(moveit_plan->move())
    // move to the pre-defined pose
        continue;
  }
  moveit_plan->stop();

  return 0;
}