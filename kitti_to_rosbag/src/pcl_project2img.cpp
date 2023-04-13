#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <pcl/common/io.h>
#include "tinycolormap.hpp"
using namespace cv; 
using namespace std;

int frame = 0;
const char* source_window = "Source image";

Eigen::MatrixXd ReadData(std::istream & data, int a, int b)
{
	Eigen::MatrixXd m_matrix(a, b);
	Eigen::VectorXd hang(a);
	for (int j = 0; j < a; j++)//共a 行
	{
		for (int i = 0; i < b; i++)//共b 列 组成一行
		{
			data >> hang(i);
		}
		m_matrix.row(j) = hang;
	}
	return m_matrix;
}

void rgb(double ratio, Scalar &RGB)
{
    //we want to normalize ratio so that it fits in to 6 regions
    //where each region is 256 units long
    int normalized = int(ratio * 256 * 6);

    //find the region for this position
    int region = normalized / 256;

    //find the distance to the start of the closest region
    int x = normalized % 256;

    uint8_t r = 0, g = 0, b = 0;
    switch (region)
    {
    case 0: r = 255; g = 0;   b = 0;   g += x; break;
    case 1: r = 255; g = 255; b = 0;   r -= x; break;
    case 2: r = 0;   g = 255; b = 0;   b += x; break;
    case 3: r = 0;   g = 255; b = 255; g -= x; break;
    case 4: r = 0;   g = 0;   b = 255; r += x; break;
    case 5: r = 255; g = 0;   b = 255; b -= x; break;
    }
    RGB << r, g, b;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "project_point");
    ros::NodeHandle nh;

    string folder_dir = argv[1];
    std::ifstream fin(folder_dir +"/intrinsic.txt", std::ios::in);
    Eigen::MatrixXd P2 = ReadData(fin, 3, 3);
    std::ifstream in(folder_dir +"/extrinsic.txt", std::ios::in);
    Eigen::MatrixXd Tr = ReadData(in, 4, 4);
    std::ifstream dis(folder_dir +"/distortion.txt", std::ios::in);
    Eigen::MatrixXd Dis = ReadData(dis, 1, 5);
    double k1 = Dis(0);
    double k2 = Dis(1);
    double p1 = Dis(2);
    double p2 = Dis(3);
    double k3 = Dis(4);
    std::cout << "P2: " << P2 << endl;
    std::cout << "Tr: " << Tr << endl;
    std::cout << "k: " << k1 << " " << k2 << " " << p1 << " " << p2 << " " << k3 <<endl;
    ros::Rate rate(1000.0);

    // for (int i = 0; i < 10; i++)
    // {
    //     ros::spinOnce();
    // }
    while(ros::ok())
    {
        ros::spinOnce();
        cv::String img_path = folder_dir +"/image_result/image3/";
        char temp[64] ;
        sprintf(temp, "%d", (int)(frame+11));
        cv::String readpath = img_path + temp + ".jpg";
        cv::Mat image = cv::imread(readpath);
        cv::Mat image_mask;
        image.copyTo(image_mask);
        char temp_lidar[64] ;
        sprintf(temp_lidar, "%d", (int)(frame));
        cv::String pcl_path = folder_dir +"/lidar/" + temp_lidar + ".bin";
        cout << "pcl_path: " << pcl_path << endl;
        std::ifstream file(pcl_path, ios::in | std::ios::binary);
        for (size_t i = 0; !file.eof(); i++) 
        {
            pcl::PointXYZI point;
            file.read((char*)&point.x, 3 * sizeof(float));
            Eigen::Vector4d location_vel_4;
            location_vel_4 << point.x, point.y, point.z, 1.0;
            // cout << "location_vel_4 " << location_vel_4.transpose() << std::endl;
            Eigen::Vector4d location_cam_4;
            location_cam_4 = Tr * location_vel_4;
            Eigen::Vector2d location_cam_norm;
            location_cam_norm << location_cam_4(0)/location_cam_4(2), location_cam_4(1)/location_cam_4(2);
            double r = location_cam_norm.norm();
            Eigen::Vector2d location_cam_after_distort;
            location_cam_after_distort(0) = location_cam_norm(0) * (1 + k1*pow(r,2) + k2* pow(r,4) + k3*pow(r,6)) + 2. * p1 * location_cam_norm(0) * location_cam_norm(1) + p2*(pow(r,2) + 2*location_cam_norm(0)*location_cam_norm(0));
            location_cam_after_distort(1) = location_cam_norm(1) * (1 + k1*pow(r,2) + k2* pow(r,4) + k3*pow(r,6)) + 2. * p2 * location_cam_norm(0) * location_cam_norm(1) + p1*(pow(r,2) + 2*location_cam_norm(1)*location_cam_norm(1));
            double u = P2(0,0) * location_cam_after_distort(0) + P2(0,2);
            double v = P2(1,1) * location_cam_after_distort(1) + P2(1,2);
            // Eigen::Vector3d location_pixel;

            // location_pixel = P2 * Tr * location_vel_4;
            // // cout << "location_pixel " << location_pixel.transpose() << std::endl;
            // location_pixel(0) = location_pixel(0)/ abs(location_pixel(2));
            // location_pixel(1) = location_pixel(1)/ abs(location_pixel(2));
            // location_pixel(2) = location_pixel(2);
            double range = sqrt(location_cam_4(0)*location_cam_4(0) + location_cam_4(1)*location_cam_4(1) + location_cam_4(2)*location_cam_4(2));
            Scalar point_rgb;
            const tinycolormap::Color color_rgb = tinycolormap::GetColor(range/100., tinycolormap::ColormapType::Jet);
            point_rgb << color_rgb.r() * 255, color_rgb.g()*255, color_rgb.b()*255;
            // point_rgb << 0, 0, 255;
            // std::cout << "pixel: " << location_pixel(0) << " " << location_pixel(1) << std::endl;
            cv::circle(image_mask, cv::Point(u, v), 1, point_rgb, -1);
        }
        file.close();
        float alpha = 0.6;
        cv::addWeighted(image, 1-alpha, image_mask, alpha,0,image);
        namedWindow( source_window);
        imshow( source_window, image);
        waitKey();
        frame++;
        rate.sleep();
    }

    
    return 0;
}