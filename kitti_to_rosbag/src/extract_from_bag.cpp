#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
using namespace cv; 
using namespace std;

// hello realsense 


cv_bridge::CvImagePtr cv_ptr;
Mat color(Size(848, 480), CV_8UC3);
string jpg_dir;
string pcl_dir;
int counting_img = 0;
int counting_pcl = 0;
bool write_jpg = true;
bool write_pcl = true;

// void color_cb(const sensor_msgs::CompressedImage::ConstPtr &msg)
// {   
//     try
//     {
//        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }
//     color = cv_ptr->image;
//     char temp[64] ;
//     sprintf(temp, "%d", (int)counting_img);
//     string log_file = jpg_dir + string(temp) + ".jpg";
//     imwrite(log_file, color);
//     counting_img++;
// }

void color_cb(const sensor_msgs::Image::ConstPtr &msg)
{   
    try
    {
       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    color = cv_ptr->image;
    char temp[64] ;
    sprintf(temp, "%d", (int)counting_img);
    string log_file = jpg_dir + string(temp) + ".jpg";
    imwrite(log_file, color);
    counting_img++;
}

void lidar_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in) 
{   
    char temp[64] ;
    sprintf(temp, "%d", (int)counting_pcl);
    string bin_file = pcl_dir+ string(temp) + ".bin";
    ofstream bin_out;
    bin_out.open(bin_file, ios::out | ios::binary);
    for(int i = 0; i < msg_in->point_num; i++)
    {   
        bin_out.write((char*)&msg_in->points[i].x, 3 * sizeof(float));
        // bin_out.write((char*)&msg_in->points[i].reflectivity, sizeof(float));
    }
    bin_out.close();
    counting_pcl++;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "extract_bag");
    ros::NodeHandle nh;

    jpg_dir = argv[1];
    pcl_dir = argv[2];
    std::cout << jpg_dir << std::endl;
    // String sub_topic = argv[3];
    ros::Subscriber color_sub = nh.subscribe("/left_camera/image", 1000, color_cb);
    // ros::Subscriber lidar_sub= nh.subscribe("/livox/lidar", 1000, lidar_cbk);

    ros::Rate rate(1000.0);

    // for (int i = 0; i < 10; i++)
    // {
    //     ros::spinOnce();
    // }
    while(ros::ok())
    {
        ros::spinOnce();
        // imshow(OPENCV_WINDOW, color);
        // blob_detect(0, 0);
        // imshow(OPENCV_WINDOW, gray_);
        // waitKey();
        rate.sleep();
    }

    
    return 0;
}