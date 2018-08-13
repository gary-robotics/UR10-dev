#include <moveit/move_group_interface/move_group_interface.h>
#include "grasp_interface/RCGripperCommand.h"


int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "gripper_plan");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    gripper.setGoalJointTolerance(0.2);
    gripper.setStartStateToCurrentState();
    
    /*
    ros::Publisher pub_= nh.advertise<grasp_interface::RCGripperCommand>("grip_command",1);
    grasp_interface::RCGripperCommand gp;
    ros::Duration(1).sleep();

    gp.position = 255;
    gp.force = 50;
    gp.speed = 100;
    pub_.publish(gp);
    ros::Duration(2).sleep();
    
    gp.position = 0;
    gp.speed = 100;
    pub_.publish(gp);
    ros::Duration(2).sleep();
    */
    
    
    gripper.setNamedTarget("close");
    gripper.move();
    ros::Duration(5).sleep();
    
    gripper.setNamedTarget("open");
    gripper.move();
    ros::Duration(5).sleep();
    


    return 0;
    
}