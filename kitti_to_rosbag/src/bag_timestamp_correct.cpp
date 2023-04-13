#include <ros/ros.h>
#include <rosbag/bag.h>
#include <iostream>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosgraph_msgs/Clock.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

rosbag::Bag bag_;
rosgraph_msgs::Clock bag_cur_time;

void image_cbk(const sensor_msgs::CompressedImage::ConstPtr &msg_in) 
{
    sensor_msgs::CompressedImage::Ptr msg(new sensor_msgs::CompressedImage(*msg_in));
    ros::Time time = msg->header.stamp;
    bag_.write("/camera/image_color/compressed", time, *msg);
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    // ros::Time time = bag_cur_time.clock;
    ros::Time time = msg->header.stamp;
    bag_.write("/livox/imu", time, *msg);
}

// void lidar_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in) 
// {
//     sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2(*msg_in));
//     // ros::Time time= bag_cur_time.clock;
//     ros::Time time = msg->header.stamp;
//     bag_.write("/livox/lidar", time, *msg);
// }

void lidar_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in) 
{
    livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in));
    // ros::Time time= bag_cur_time.clock;
    ros::Time time = msg->header.stamp;
    bag_.write("/livox/lidar", time, *msg);
}

void clock_cbk(const rosgraph_msgs::Clock::ConstPtr &msg_in)
{   
    bag_cur_time = *msg_in;
}



int main(int argc, char** argv)
{   
    ros::init(argc, argv, "timestamp_correct");
    ros::NodeHandle nh;
    ros::Subscriber image_sub = nh.subscribe("/camera/image_color/compressed", 1000, image_cbk);
    ros::Subscriber imu_sub= nh.subscribe("/livox/imu", 1000, imu_cbk);
    ros::Subscriber lidar_sub= nh.subscribe("/livox/lidar", 1000, lidar_cbk);
    ros::Subscriber clock_sub= nh.subscribe("/clock", 1000, clock_cbk);


    std::string output_filename = argv[1];
    bag_.open(output_filename, rosbag::bagmode::Write);
    ros::spin();
    return 0;
}