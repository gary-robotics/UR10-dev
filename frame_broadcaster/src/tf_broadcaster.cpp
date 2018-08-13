#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#define PI 3.1415926

int main(int argc ,char** argv)
 {
	ros::init(argc,argv,"robot_tf_publisher");
	ros::NodeHandle n;
	
	ros::Rate r(50);
	tf::TransformBroadcaster broadcaster1;
	tf::TransformBroadcaster broadcaster2;
	while(n.ok()){
	/*
	broadcaster1.sendTransform(
	  tf::StampedTransform(  // x y z w 
		tf::Transform(tf::Quaternion(0.0236331257289576,0.999567920046238,0.0172280100194881,0.00294011113491014),
		tf::Vector3(0.730361779589858,-0.63665209974787,0.914470497824103)),                        
                ros::Time::now(),"camera_rgb_optical_frame","base")); 
	*/
		/*HK_kinect w x y z*/
		broadcaster2.sendTransform(
	  tf::StampedTransform(  // x y z w 
		tf::Transform(tf::Quaternion(-0.122919601861648,0.732033580840784,-0.659760569807942,0.117190437008716),
		tf::Vector3(0.278292947573651,1.32699493388865,0.26265011950633)),                        
                ros::Time::now(),"base","camera_rgb_optical_frame")); 

		r.sleep();
	}
}


/*
 this is the origin handeye calibration result from matlab
竖直
  0.136799642673473,0.990041305598571,0.03113976256937,0.0115925044839447    x y z w
 0.595330588115464,-0.78957707502109,0.918197182130532  x y z
水平
  -0.120606643208955,0.733048988572894,-0.659401595753422,-0.115250828560863    x y z w
 0.702122716331048,0.134075972213284,1.18346051530078  x y z

 */
