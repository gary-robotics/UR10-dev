#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

double cx = 325.5;
double cy = 253.5;
double fx = 518.0;
double fy = 519.0;
double depthScale = 1.0;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class ImageToPointCloud
{
public:
    ImageToPointCloud(ros::NodeHandle &nh)
    {
        color_sub_ =  new message_filters::Subscriber<sensor_msgs::Image>(nh,"camera/rgb/image_raw",10);
        depth_sub_ =  new message_filters::Subscriber<sensor_msgs::Image>(nh,"camera/depth_registered/hw_registered/image_rect",10);
        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *color_sub_, *depth_sub_);
        sync->registerCallback(boost::bind(&ImageToPointCloud::callback,this,_1,_2));
        pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("test", 100);
    }

    void callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth)
    {
        ROS_INFO_STREAM("in callback");

        /* receive image msg */
        try
        {
            color_ptr = cv_bridge::toCvShare(color, sensor_msgs::image_encodings::BGR8);
            depth_ptr = cv_bridge::toCvShare(depth, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        /* generate pointcloud */
        for(int i=0;i<color_ptr->image.rows;++i)
        {
            for(int j=0;j<color_ptr->image.cols;++j)
            {
                unsigned int d = depth_ptr->image.at<uchar>(i,j);
                //if ( d==0 ) continue;
                Eigen::Vector3d point;
                point[2] = double(d)/depthScale; 
                point[0] = (j-cx)*point[2]/fx;
                point[1] = (i-cy)*point[2]/fy;  
                p.x = point[0];
                p.y = point[1];
                p.z = point[2];
                p.b = color_ptr->image.data[ i*color_ptr->image.step+j*color_ptr->image.channels() ];
                p.g = color_ptr->image.data[ i*color_ptr->image.step+j*color_ptr->image.channels()+1 ];
                p.r = color_ptr->image.data[ i*color_ptr->image.step+j*color_ptr->image.channels()+2 ];
                pointCloud_.points.push_back(p);
            }
        }
        pointCloud_.is_dense = false;

        /* change to ROS PointCloud2 */
        pcl::toROSMsg(pointCloud_, pt2_);

        /* ROS Header after chaning */
        pt2_.header.frame_id = "camera_rgb_optical_frame";
        pt2_.header.stamp = ros::Time::now();

        /* publish */
        pointcloud_pub_.publish(pt2_);
        pointCloud_.points.clear();  
        
       
    }   
private:
    message_filters::Subscriber<sensor_msgs::Image> *color_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;
    message_filters::Synchronizer<MySyncPolicy> *sync;
    cv_bridge::CvImageConstPtr color_ptr;
    cv_bridge::CvImageConstPtr depth_ptr;
    PointCloud pointCloud_;
    pcl::PointXYZRGB p;
    sensor_msgs::PointCloud2 pt2_;
    ros::Publisher pointcloud_pub_;
};


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"convert_rgbd_image_to_pointcloud");
    ros::NodeHandle nh;
    ImageToPointCloud image_to_pointcloud(nh);
    ros::Duration(2).sleep();
    ros::AsyncSpinner spinner(2);
    spinner.start();
    while(ros::ok());

    return 0;
}