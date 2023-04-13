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
#include "DBSCAN_kdtree.h"
using namespace cv; 
using namespace std;

int frame = 0;
const char* source_window = "Source image";

struct track_label_s
{
    int             frame_id;
    int             track_id;
    string          type;
    double          truncated;
    int             occluded;
    double          alpha;
    Eigen::Vector4d pixel;
    Eigen::Vector3d dimension;
    Eigen::Vector3d location;
    double          rotation;
    double          score;
    std::vector<Eigen::Vector2d> points_pixel;
    pcl::PointCloud<pcl::PointXYZI> points_cloud;
};

void GetClusterResult(pcl::PointCloud<pcl::PointXYZI>::Ptr points_in, std::vector<pcl::PointIndices> &cluster_indices)
{   
    int nn_points_size = 2;
    double nn_points_radius = 0.5;
    int min_cluster_size = 5;
    int max_cluster_size = 10000;
    if(points_in->size()<2)
    {
        return;
    }
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(points_in);
    DBSCANKdtreeCluster<pcl::PointXYZI> ec;
    ec.setCorePointMinPts(nn_points_size);
    ec.setClusterTolerance(nn_points_radius);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(points_in);
    ec.extract(cluster_indices);
}

void write_track(std::ofstream &output, track_label_s &track)
{
    output << track.frame_id << ' ' << track.track_id << ' ' << track.type << ' ' << track.truncated << ' ';
    output << track.occluded << ' ' << track.alpha << ' ' << track.pixel(0) << ' ' << track.pixel(1) << ' ';
    output << track.pixel(2) << ' ' << track.pixel(3) << ' ' << track.dimension(0) << ' ' << track.dimension(1) << ' ';
    output << track.dimension(2) << ' ' << track.location(0) << ' ' << track.location(1) << ' ' << track.location(2) << ' ';
    output << track.rotation << ' ' << track.score << endl;
}

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

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "gt_revision");
    ros::NodeHandle nh;

    string folder_dir = argv[1];
    string input_track = argv[2];
    string revise_track = argv[3];
    string image_start = argv[4];
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

    // READ and RECORD input track information
    std::vector<std::vector<track_label_s>> track_vector_by_frame(5000); 
    std::ifstream inputfile(input_track, std::ios::in);  //读取    
    std::string line;
    while (std::getline(inputfile, line)) 
    {
        std::stringstream line_stream(line);
        std::string frame;
        std::getline(line_stream, frame, ' ');
        std::string track_id;
        std::getline(line_stream, track_id, ' ');
        std::string class_type;
        std::getline(line_stream, class_type, ' ');
        int frame_i = std::stoi(frame);
        int index = std::stoi(track_id);
        std::vector<double> parsed_doubles;
        while (!line_stream.eof()) {
            std::string element;
            std::getline(line_stream, element, ' ');
            if (element.empty()) {
                continue;
            }
            try {
                parsed_doubles.emplace_back(std::stod(element));
            } catch (const std::exception& exception) {
                std::cout << "Could not parse number in import file.\n";
            }
        }
        Eigen::Matrix<double, 15, 1> V(parsed_doubles.data());  //读取坐标并输入
        Eigen::Vector3d location_vel(V[0], V[1], V[2]);
        Eigen::Vector3d dimension_vel(V[3], V[4], V[5]);
        Eigen::Quaterniond quaternion_vel(V[6], V[7], V[8], V[9]);
        track_label_s track;
        track.frame_id = frame_i;
        track.track_id = index;
        track.type = class_type;
        track.truncated = V[0];
        track.occluded = V[1];
        track.alpha = V[2];
        track.pixel << V[3], V[4], V[5], V[6];
        track.dimension << V[7], V[8], V[9];
        track.location << V[10], V[11], V[12];
        track.rotation = V[13];
        track.score = V[14];
        track_vector_by_frame[frame_i].push_back(track);
    }

    // for (int i = 0; i < 10; i++)
    // {
    //     ros::spinOnce();
    // }
    std::ofstream outputfile(revise_track, std::ios::out);  //读取  
    while(ros::ok())
    {
        ros::spinOnce();
        cv::String img_path = folder_dir +"/image_result/image4/";
        char temp[64] ;
        sprintf(temp, "%d", (int)(frame+11+stoi(image_start)));
        int current_frame = (int)(frame+11);
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
            double range = sqrt(location_cam_4(0)*location_cam_4(0) + location_cam_4(1)*location_cam_4(1) + location_cam_4(2)*location_cam_4(2));
            Scalar point_rgb;
            const tinycolormap::Color color_rgb = tinycolormap::GetColor(range/10., tinycolormap::ColormapType::Jet);
            point_rgb << color_rgb.r() * 255, color_rgb.g()*255, color_rgb.b()*255;
            cv::circle(image_mask, cv::Point(u, v), 1, point_rgb, -1);

            for(int k = 0; k < track_vector_by_frame[current_frame].size(); k++)
            {   
                track_label_s & current_track = track_vector_by_frame[current_frame][k];
                if(u > current_track.pixel(0) && u < current_track.pixel(2) && v > current_track.pixel(1) && v < current_track.pixel(3))
                {   
                    Eigen::Vector2d pixel(u,v);
                    current_track.points_pixel.push_back(pixel);
                    current_track.points_cloud.push_back(point);
                }
            }
        }
        file.close();

        //Revise and Rewrite track information  
        for(int k = 0; k < track_vector_by_frame[current_frame].size(); k++)
        {   
            track_label_s & current_track = track_vector_by_frame[current_frame][k];
            double width_pixel = current_track.pixel(2) - current_track.pixel(0);
            double height_pixel = current_track.pixel(3) - current_track.pixel(1);
            if(current_track.points_pixel.size() > 4)
            {   
                std::vector<pcl::PointIndices> points_indices;
                GetClusterResult(current_track.points_cloud.makeShared(), points_indices);
                double max_u, max_v, min_u, min_v;
                if(points_indices.size() > 0)
                {
                    min_u= current_track.points_pixel[points_indices[0].indices.at(0)](0), max_u = current_track.points_pixel[points_indices[0].indices.at(0)](0);
                    min_v= current_track.points_pixel[points_indices[0].indices.at(0)](1), max_v = current_track.points_pixel[points_indices[0].indices.at(0)](1);
                    for (int ind = 0; ind < points_indices.size(); ind++)
                    {   
                        for(int p = 0; p < points_indices[ind].indices.size(); p++)
                        {   
                            int i = points_indices[ind].indices.at(p);
                            double u = current_track.points_pixel[i](0);
                            double v = current_track.points_pixel[i](1);
                            if(u < min_u) min_u = u;
                            if(u > max_u) max_u = u;
                            if(v < min_v) min_v = v;
                            if(v > max_v) max_v = v;
                        }
                    }
                    current_track.pixel(0) = min_u;
                    current_track.pixel(1) = min_v;
                    current_track.pixel(2) = max_u;
                    current_track.pixel(3) = max_v;
                    if((max_u - min_u) > 0.2 * width_pixel && (max_v - min_v) > 0.2 * height_pixel)
                    {
                        write_track(outputfile, current_track);
                        cv::Point pt1((int)min_u, (int)min_v);
                        cv::Point pt2((int)max_u, (int)max_v);
                        cv::rectangle(image_mask, pt1, pt2, cv::Scalar(255,0,0), 2);
                    }
                }
            }
            
        }

        float alpha = 0.6;
        cv::addWeighted(image, 1-alpha, image_mask, alpha,0,image);
        namedWindow( source_window);
        imshow( source_window, image);
        waitKey();
        frame++;
        rate.sleep();
    }
    outputfile.close();

    
    return 0;
}