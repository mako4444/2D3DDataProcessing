#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <string>

void laser_cloud_surround_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    ROS_INFO("I heard [%d] Points", cloud.points.size());
    
    if (cloud.points.size() > 400000)
    {   
        //std::string fielName = "/Data/Robotcar_radar/Scenes/" + std::to_string(msg->header.stamp.sec) + ".pcd";
        std::string fielName = std::to_string(cloud.points.size()) + ".pcd";
        pcl::io::savePCDFileASCII(fielName, cloud);
    }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "surroundcloud2pcd_listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/laser_cloud_surround", 2, laser_cloud_surround_callback);

    ros::spin();
}