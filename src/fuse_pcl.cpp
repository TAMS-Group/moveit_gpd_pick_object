#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

class FusePCL
{
public:
  FusePCL(ros::NodeHandle &nh) : kinect1_sub_(nh, "/camera/depth_registered/points", 1), 
                                 kinect2_sub_(nh, "/kinect2/sd/points", 1),
                                 sync(MySyncPolicy(10), kinect1_sub_, kinect2_sub_)
  {
    sync.registerCallback(boost::bind(&FusePCL::kinect_callback, this, _1, _2));
    pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("fused_points", 10);
  }

private:
  void kinect_callback(const sensor_msgs::PointCloud2::ConstPtr& kinect1_msg, 
                       const sensor_msgs::PointCloud2::ConstPtr& kinect2_msg)
  {
    sensor_msgs::PointCloud2 transformed_kinect1_msg, transformed_kinect2_msg, output_msg;
    
    // transform both point clouds to the same frame.
    listener_.waitForTransform("table_top", kinect1_msg->header.frame_id, kinect1_msg->header.stamp, ros::Duration(1.0));
    listener_.waitForTransform("table_top", kinect2_msg->header.frame_id, kinect2_msg->header.stamp, ros::Duration(1.0));
    pcl_ros::transformPointCloud("table_top", *kinect1_msg, transformed_kinect1_msg, listener_);
    pcl_ros::transformPointCloud("table_top", *kinect2_msg, transformed_kinect2_msg, listener_);
    pcl::PointCloud<pcl::PointXYZRGBA> kinect1_pcl, kinect2_pcl, output_pcl;

    pcl::fromROSMsg(transformed_kinect1_msg, kinect1_pcl);
    pcl::fromROSMsg(transformed_kinect2_msg, kinect2_pcl);
    if(kinect1_pcl.size() == 0 || kinect2_pcl.size() == 0)
      return;
    output_pcl = kinect1_pcl + kinect2_pcl;
    pcl::toROSMsg(output_pcl, output_msg);

    pc_pub_.publish(output_msg);
  }
 
  message_filters::Subscriber<sensor_msgs::PointCloud2> kinect1_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> kinect2_sub_;
  message_filters::Synchronizer<MySyncPolicy> sync;
  ros::Publisher pc_pub_;
  tf::TransformListener listener_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fuse_pcl");
  ros::NodeHandle nh;
  FusePCL f_pcl(nh);
  while(ros::ok())
    ros::spin();
  return 0;
}
