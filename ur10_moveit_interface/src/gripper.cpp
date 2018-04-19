#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "gripper_plan");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    gripper.setNamedTarget("close");
    gripper.move();
    gripper.setNamedTarget("open");
    gripper.move();


    return 0;
    
}