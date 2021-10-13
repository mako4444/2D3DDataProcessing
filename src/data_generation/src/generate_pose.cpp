#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <string>
#include <fstream>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "generate_pose_listener");

    ros::NodeHandle n;

    tf::TransformListener listener;

    ros::Time lastTimestamp = ros::Time::now();

    ros::Rate rate(10.0);

    std::ofstream fout("pose.txt");

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

        if (transform.stamp_ == lastTimestamp)
        {
            continue;
        }
        
        ROS_INFO("Timestamp [] camera pose in map x = %f, y = %f, z = %f", 
                transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

        // record data to txt ile.
        // format: sim_time, translation_x, translation_y, translation_z, rotation_x, rotation_y, rotation_z, rotation_w
        fout << transform.stamp_.toNSec() / 1000 << " " // convert from nanosecond to microsecond
            << transform.getOrigin().x() << " "
            << transform.getOrigin().y() << " "
            << transform.getOrigin().z() << " "
            << transform.getRotation().x() << " "
            << transform.getRotation().y() << " "
            << transform.getRotation().z() << " "
            << transform.getRotation().w() << std::endl;

        lastTimestamp = transform.stamp_;
    }

    fout.close();
}