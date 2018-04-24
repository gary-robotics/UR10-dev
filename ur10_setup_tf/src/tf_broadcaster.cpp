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
		tf::Transform(tf::Quaternion(0.724653150487702,0.689015272988343,-0.000038399023251,0.011651763924937),
		tf::Vector3(1.236198544416497, 0.243724848741023, 0.958845055486651)),
                ros::Time::now(),"base","camera_link"));
				r.sleep();
	}



//0.009950416684597,0.710376335014308 , -0.703749771718084 , -0.001646410091099 
//0.710376335014308 , -0.703749771718084 , -0.001646410091099 ,  0.009950416684597
}

/*
 this is the origin handeye calibration result from matlab
  0.011651763924937   0.724653150487702   0.689015272988343  -0.000038399023251    w x y z
 1.236198544416497  x
 0.243724848741023  y
 0.958845055486651  z
 */
