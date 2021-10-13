#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

/**
 * @brief expects a file with a list of timestamps (<seconds> <nanoseconds>).
 */
std::vector<ros::Time> LoadTimestamps(const std::string& filename)
{
    std::vector<ros::Time> timestamps;
    std::ifstream ifs(filename.c_str());
    std::string line;
    std::string delimiter = " ";

    while (std::getline(ifs, line))
    {
        string unix_time = line.substr(0, line.find(delimiter)); 
        string strSec = unix_time.substr(0, 10);
        string strNsec = unix_time.substr(10, 16);
        strNsec.append("000");

        uint32_t sec = atoi(strSec.c_str());
        uint32_t nsec = atoi(strNsec.c_str());

        timestamps.push_back(ros::Time(sec, nsec));
    }

    return timestamps;
}

std::vector<string> LoadUnixTimestamps(const std::string& filename, std::string& startTime, std::string& endTime)
{
    std::vector<string> unix_timestamps;
    std::ifstream ifs(filename.c_str());
    std::string line;
    std::string delimiter = " ";

    int startTimeSec = 0;
    int endTimeSec = 0;

    if (startTime == "" || endTime == "")
    {
        startTimeSec = -1;
        endTimeSec = INT_MAX;
    }
    else
    {
        startTimeSec = atoi(startTime.substr(0, 10).c_str());
        endTimeSec = atoi(endTime.substr(0, 10).c_str());
    }

    while (std::getline(ifs, line))
    {
        string unix_time = line.substr(0, line.find(delimiter)); 
        string strSec = unix_time.substr(0, 10);
        if (atoi(strSec.c_str()) >= startTimeSec && atoi(strSec.c_str()) <= endTimeSec)
        {
            unix_timestamps.push_back(unix_time);
        }
    }

    startTime = unix_timestamps[0];
    endTime = unix_timestamps[unix_timestamps.size()-1];

    return unix_timestamps;
}

ros::Time UnixToRosTimestamp(string unix_timestamp)
{   
    string strSec = unix_timestamp.substr(0, 10);
    string strNsec = unix_timestamp.substr(10, 16);
    strNsec.append("000");

    uint32_t sec = atoi(strSec.c_str());
    uint32_t nsec = atoi(strNsec.c_str());

    return ros::Time(sec, nsec);
}

typedef struct PointXYZI
{
    float x;
    float y;
    float z;
    float i;
} ReadPoint;

/*
arg[1] : The path of bin files.
arg[2] : The timestamp file path.
arg[3] : The generated bag file path.
arg[4] : Start time, Unix time.
arg[5] : End time, Unix time.
*/
int main(int argc, char **argv)
{
    if (argc != 4 && argc != 6)
    {
        cout << "Invalid arguments." << endl;
        return EXIT_FAILURE;    
    }

    string strDatasetPath = argv[1];
    string strTimestampFilePath = argv[2];
    string strBagFilePath = argv[3];

    string strStartTime;
    string strEndtime;
    if (argc == 6)
    {
        strStartTime = argv[4];
        strEndtime = argv[5];
    }

    string strPlaceholder;
    cin >> strPlaceholder;
 //   ros::init(argc, argv, "converter");
 //   ros::NodeHandle n;

    // Traverse all timestamps in the timestamp file
    std::vector<string> unixTimestamps = LoadUnixTimestamps(strTimestampFilePath, strStartTime, strEndtime);

    // Init ros bag
    rosbag::Bag bag;

    bag.open(strBagFilePath, rosbag::bagmode::Write);

    // Use timestamps to find corresponding .bin files in given laser data folder of robotcar
    // For each .bin and timestamp pair
    size_t seq = 0;
    fstream fs;

    int bufferSize = sizeof(ReadPoint);
    char buffer[bufferSize];

    for (auto time : unixTimestamps)
    {
        // Load all points from the bin file to a pcl::pointcloud instance
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud->is_dense = false;
        cloud->height = 1;
        
        string lidarFilePath = strDatasetPath + "/" + time + ".bin";
        std::ifstream fs(lidarFilePath, std::ios::in | std::ios::binary | ios::ate);
        if (!fs)
            return EXIT_FAILURE;

        std::cout << "Reading "<< time + ".bin";

        int filesize = fs.tellg();

        std::cout << "; File size "<< filesize << " Bytes";
        fs.clear();
        fs.seekg(0, std::ios::beg);

        int point_num = (filesize / 4) / 4;

        pcl::PointXYZI point;

        float fx[1];

        int pointCount = 0;
        while (pointCount < point_num  && fs.read((char*)fx, 4))
        {
            point.x = fx[0];

            cloud->push_back(point);
            ++pointCount;
        }

        float fy[1];
        for (int i = 0; i < point_num; i++)
        {
            fs.read((char*)fy, 4);
            cloud->points[i].y = fy[0];
        }

        float fz[1];
        for (int i = 0; i < point_num; i++)
        {
            fs.read((char*)fz, 4);
            cloud->points[i].z = fz[0];
        }

        float fi[1];
        for (int i = 0; i < point_num; i++)
        {
            fs.read((char*)fi, 4);
            cloud->points[i].intensity = fi[0];
        }

/*
        for (int j = 0; j < cloud->points.size(); ++j)
        {
            cout << "Point " << j 
            << " x=" << cloud->points[j].x
            << " y=" << cloud->points[j].y
            << " z=" << cloud->points[j].z
            << " i=" << cloud->points[j].intensity << "\n";
        }
    */
        cout << " " << cloud->points.size() << " Points \n";
        cloud->width = cloud->points.size();

        ros::Time timestamp = UnixToRosTimestamp(time);

        // Convert the pointcloud instance to sensor_msg/PointCloud2 with pcl::toROSMsg
        sensor_msgs::PointCloud2 pointcloud_msg;
        pcl::toROSMsg(*cloud, pointcloud_msg);
        pointcloud_msg.header.stamp = timestamp;
        pointcloud_msg.header.seq = seq++;
        pointcloud_msg.header.frame_id = "velodyne";
        /*
        sensor_msgs::PointField int16placeholder;
        int16placeholder.name = "ring";
        int16placeholder.offset = 20;
        int16placeholder.datatype = sensor_msgs::PointField::UINT16;
        int16placeholder.count = 1;
        pointcloud_msg.fields.push_back(int16placeholder);
        */

        // Add the PointCloud2 message to the rosbag
        bag.write("/velodyne_points", timestamp, pointcloud_msg);

        std::cout << "Wrote scan " << seq - 1 << "\n";

/*
        if (seq >= 1000)
        {
            break;
        }*/
    }

    bag.close();

    return 0;
}