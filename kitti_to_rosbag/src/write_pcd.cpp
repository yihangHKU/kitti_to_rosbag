#include <ros/ros.h>
#include <rosbag/bag.h>
#include <iostream>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr total_points(new pcl::PointCloud<pcl::PointXYZI>());
int pcl_save_num = 0;
using namespace std;

void lidar_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in) 
{
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2(*msg_in));
    pcl::PointCloud<pcl::PointXYZI> current_points;
    pcl::fromROSMsg(*msg, current_points);
    *total_points += current_points;
    pcl_save_num ++;
    std::cout << pcl_save_num << std::endl;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "write_pcd");
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub= nh.subscribe("/livox/lidar", 1000, lidar_cbk);
    string ROOT_DIR = argv[1];
    int pcd_save_frame = 80;
    std::cout << "pcd_save_frame: " << pcd_save_frame << std::endl;
    string all_points_dir(string(string(ROOT_DIR)) + string(".pcd"));
    ros::Rate rate(100.0);
    while(ros::ok())
    {   
        ros::spinOnce();
        if(pcl_save_num == pcd_save_frame)
        {
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to " << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *total_points);  
        }
        rate.sleep();
    }
    

    ros::spin();
    return 0;
}