#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
int main(int argc ,char** argv)
 {
	ros::init(argc,argv,"robot_tf_publisher");
	ros::NodeHandle n;
	
	ros::Rate r(50);
	tf::TransformBroadcaster broadcaster;
	while(n.ok()){
	broadcaster.sendTransform(
	  tf::StampedTransform(  // x y z w 
		tf::Transform(tf::Quaternion(0.0183800779366377,0.999782858946973,0.00864747869630508,-0.00465067769883286),   //tf::Transform(tf::Quaternion(0.7198,0.6890,-0.0568,-0.0627),
		tf::Vector3(0.751787452293726,-0.611144770753265,0.912557635583905)),                          //tf::Vector3(1.0724, 0.3202, 0.9910)),
                ros::Time::now(),"camera_rgb_optical_frame","base"));         //ros::Time::now(),"world","camera_rgb_optical_frame"));
				r.sleep();
	}

}


/*
 this is the origin handeye calibration result from matlab
  0.011651763924937   0.724653150487702   0.689015272988343  -0.000038399023251    w x y z
 1.236198544416497  x
 0.243724848741023  y
 0.958845055486651  z
 */
