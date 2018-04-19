#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
int main(int argc ,char** argv)
 {
	ros::init(argc,argv,"robot_tf_publisher");
	ros::NodeHandle n;
	
	ros::Rate r(50);
	tf::TransformBroadcaster broadcaster1;
	tf::TransformBroadcaster broadcaster2;
	while(n.ok()){
	broadcaster1.sendTransform(
	  tf::StampedTransform(  // x y z w 
		tf::Transform(tf::Quaternion(0.7108,0.7030,0.0109,0.0226),
		tf::Vector3(1.380737478650816, 0.220867800269379, 0.883465239696023)),
                ros::Time::now(),"base","camera_link"));
	}



//0.009950416684597,0.710376335014308 , -0.703749771718084 , -0.001646410091099 
//0.710376335014308 , -0.703749771718084 , -0.001646410091099 ,  0.009950416684597
}

/*
 this is the origin handeye calibration result from matlab
 0.0226    0.7108    0.7030    0.0109    w x y z
 1.38073747865082   x
 0.220867800269379  y
 0.883465239696023  z
 */
