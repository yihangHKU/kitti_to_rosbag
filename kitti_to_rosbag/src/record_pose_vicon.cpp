#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <ros/ros.h>
#include <Eigen/Core>
// #include <common_lib.h>-
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>

geometry_msgs::PoseStamped vicon_pose;
std::string vicon_file = "/media/yihang/LYH/livox/vicon/vicon.txt";
std::string lio_file = "/media/yihang/LYH/livox/vicon/lio.txt";
std::ofstream fout_vicon, fout_lio;

void vicon_cbk(const geometry_msgs::PoseStamped::ConstPtr &msg_in)
{
    vicon_pose = *msg_in;
}

void lio_cbk(const nav_msgs::Odometry::ConstPtr &msg_in)
{
    nav_msgs::Odometry lio_odom = *msg_in;
    geometry_msgs::Pose lio_pose = lio_odom.pose.pose;
    Eigen::Quaterniond lio_quaternion(lio_pose.orientation.w, lio_pose.orientation.x, lio_pose.orientation.y, lio_pose.orientation.z);
    Eigen::Matrix3d R_lio(lio_quaternion);
    Eigen::Vector3d t_lio(lio_pose.position.x, lio_pose.position.y, lio_pose.position.z);
    fout_lio << R_lio(0,0) << " " <<  R_lio(0,1) << " " << R_lio(0,2) << " " << t_lio(0) << " ";
    fout_lio << R_lio(1,0) << " " <<  R_lio(1,1) << " " << R_lio(1,2) << " " << t_lio(1) << " ";
    fout_lio << R_lio(2,0) << " " <<  R_lio(2,1) << " " << R_lio(2,2) << " " << t_lio(2) << std::endl;

    Eigen::Quaterniond vicon_quaternion(vicon_pose.pose.orientation.w, vicon_pose.pose.orientation.x, vicon_pose.pose.orientation.y, vicon_pose.pose.orientation.z);
    Eigen::Matrix3d R_vicon(vicon_quaternion);
    Eigen::Vector3d t_vicon(vicon_pose.pose.position.x, vicon_pose.pose.position.y, vicon_pose.pose.position.z);
    fout_vicon << R_vicon(0,0) << " " <<  R_vicon(0,1) << " " << R_vicon(0,2) << " " << t_vicon(0) << " ";
    fout_vicon << R_vicon(1,0) << " " <<  R_vicon(1,1) << " " << R_vicon(1,2) << " " << t_vicon(1) << " ";
    fout_vicon << R_vicon(2,0) << " " <<  R_vicon(2,1) << " " << R_vicon(2,2) << " " << t_vicon(2) << std::endl;
    std::cout << "log" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "record_pose");
    ros::NodeHandle nh;
    fout_vicon.open(vicon_file, std::ios::out);
    fout_lio.open(lio_file, std::ios::out);
    
    ros::Subscriber sub_vicon = nh.subscribe("/vrpn_client_node/mars_lidar/pose", 20000, vicon_cbk);
    ros::Subscriber sub_lio = nh.subscribe("/lidar_slam/odom", 20000, lio_cbk);
    ros::Rate rate(5000);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}