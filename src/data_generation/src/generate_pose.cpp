#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <string>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "generate_pose_listener");

    ros::NodeHandle n;

    tf::TransformListener listener;

    ros::Rate rate(10.0);
    while (n.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("map", "camera", ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }    

        ROS_INFO("I heard camera pose in map x = %f, y = %f, z = %f", 
                transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

        
    }
}