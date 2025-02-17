#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include "kitti_to_rosbag/kitti_parser.h"
#include "kitti_to_rosbag/kitti_common.h"
#include "kitti_to_rosbag/kitti_ros_conversions.h"
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

using namespace std;
Eigen::MatrixXd ReadData(std::istream & data, int a, int b)
{
	Eigen::MatrixXd m_matrix(a, b);
    cout << "here" << endl;
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "kittibbox_transfer");
    ros::NodeHandle nh;

    std::string folder_dir = argv[1];
    std::string sequence_num = argv[2];
    std::string output_file = folder_dir + "/" + sequence_num + "_final.txt";
    std::string input_file = folder_dir + "/" + sequence_num + ".txt";
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
    Eigen::Matrix3d R_cam_vel = Tr.block(0,0,3,3);
    float image_width = 1280.;
    float image_height = 1024.;

    // std::cout << "hi " << std::endl;
    // std::string sequence_num = argv[4];
    // std::string input_file = argv[1] + sequence_num + ".txt";
    // std::string output_file = argv[2] + sequence_num + ".txt";
    // std::string calib_file = argv[3] + sequence_num + ".txt";
    // std::cout << "hi " << std::endl;
    // kitti::KittiParser parser_(argv[3], "", sequence_num, true);
    // parser_.loadCalibration();
    // kitti::Transformation T_cam0_vel = parser_.T_cam0_vel().inverse();
    // kindr::minimal::RotationQuaternionTemplate<double> Rot_cam_vel = T_cam0_vel.getRotation();
    // Eigen::Quaterniond quat_cam_vel = Rot_cam_vel.toImplementation();
    // Eigen::Matrix3d R_cam_vel(quat_cam_vel);
    // std::cout << "hi " << std::endl;
    // Eigen::Vector3d t_cam_vel = T_cam0_vel.getPosition();
    // Eigen::Matrix<double, 3, 4> P2 = parser_.camera_calibrations_[0].projection_mat;
    // Eigen::Matrix3d R0_rect33 = parser_.camera_calibrations_[0].rect_mat;
    // Eigen::Matrix4d R0_rect;
    // R0_rect << R0_rect33(0,0), R0_rect33(0,1), R0_rect33(0,2), 0.0,
    //             R0_rect33(1,0), R0_rect33(1,1), R0_rect33(1,2), 0.0,
    //             R0_rect33(2,0), R0_rect33(2,1), R0_rect33(2,2), 0.0,
    //             0.0,               0.0,              0.0,       1.0;
    // Eigen::Matrix4d Tr = parser_.T_cam0_vel().getTransformationMatrix();
    // std::cout << "P2: " << P2 << std::endl;
    // std::cout << "R0_rect: " << R0_rect << std::endl;
    // std::cout << "Tr: " << Tr << std::endl;

    // 
    std::vector<std::vector<std::string>> class_vector(100000);
    std::ifstream inputfile0(input_file, std::ios::in);
    std::string line0;
    while (std::getline(inputfile0, line0)) {
        std::stringstream line_stream(line0);
        std::string frame;
        std::getline(line_stream, frame, ' ');
        std::string track_id;
        std::getline(line_stream, track_id, ' ');
        std::string class_type;
        std::getline(line_stream, class_type, ' ');
        int frame_i = std::stoi(frame);
        int index = std::stoi(track_id);
        class_vector[index].push_back(class_type);
        // if(index==361) std::cout << "frame: " << frame_i << " index: " << index << " class type: " << class_type << std::endl;
    }
    // for(int i = 0; i < class_vector.size(); i++)
    // {
    //     if(class_vector[i].empty()) continue;
    //     if(class_vector[i].size() < 3)
    //     {   
    //         class_vector[i].clear();
    //     }

    // }
    std::cout << "here: "<< std::endl;


    std::ifstream inputfile(input_file, std::ios::in);
    std::ofstream outputfile;
    outputfile.open(output_file, std::ios::out);
    std::string line;
    while (std::getline(inputfile, line)) {
        std::stringstream line_stream(line);
        std::string frame;
        std::getline(line_stream, frame, ' ');
        std::string track_id;
        std::getline(line_stream, track_id, ' ');
        std::string class_type;
        std::getline(line_stream, class_type, ' ');
        int frame_i = std::stoi(frame);
        int index = std::stoi(track_id);
        std::cout << "frame: " << frame_i << std::endl;
        std::cout << "track_id: " << index << std::endl;
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
        Eigen::Matrix<double, 11, 1> V(parsed_doubles.data());
        Eigen::Vector3d location_vel(V[0], V[1], V[2]);
        Eigen::Vector3d dimension_vel(V[3], V[4], V[5]);
        Eigen::Quaterniond quaternion_vel(V[6], V[7], V[8], V[9]);
        Eigen::Matrix3d R_vel(quaternion_vel);
        double vel = V[10];
        // Eigen::Vector3d location_cam = R_cam_vel.transpose() * (location_vel - t_cam_vel);
        Eigen::Vector3d dimentsion_cam = R_cam_vel * dimension_vel;
        Eigen::Quaterniond quaternion_cam;
        quaternion_cam = R_cam_vel * R_vel;
        Eigen::Matrix3d R_cam = R_cam_vel * R_vel;
        Eigen::Vector3d rpy = R_cam.eulerAngles(2,0,1);
        double rotation_y = rpy(0);
        std::vector<Eigen::Vector3d> vertex(8);
        vertex.resize(8);
        vertex[0] << 0.5 * dimension_vel[0], 0.5 * dimension_vel[1], 0.5 * dimension_vel[2];
        vertex[1] << -0.5 * dimension_vel[0], 0.5 * dimension_vel[1], 0.5 * dimension_vel[2];
        vertex[2] << 0.5 * dimension_vel[0], -0.5 * dimension_vel[1], 0.5 * dimension_vel[2];
        vertex[3] << 0.5 * dimension_vel[0], 0.5 * dimension_vel[1], -0.5 * dimension_vel[2];
        vertex[4] << -0.5 * dimension_vel[0], -0.5 * dimension_vel[1], 0.5 * dimension_vel[2];
        vertex[5] << -0.5 * dimension_vel[0], 0.5 * dimension_vel[1], -0.5 * dimension_vel[2];
        vertex[6] << 0.5 * dimension_vel[0], -0.5 * dimension_vel[1], -0.5 * dimension_vel[2];
        vertex[7] << -0.5 * dimension_vel[0], -0.5 * dimension_vel[1], -0.5 * dimension_vel[2];
        for (int i=0; i<8;i++)
        {
            vertex[i] = R_vel * vertex[i] + location_vel;
        }

        Eigen::Vector4d location_vel_4;
        location_vel_4 << location_vel[0], location_vel[1], location_vel[2], 1.0;
        Eigen::Vector4d location_cam_4;
        location_cam_4 = Tr * location_vel_4;
        double alpha = atan2(location_cam_4[0], location_cam_4[2]);
        std::cout << "alpha: " << alpha << std::endl;

        Eigen::Vector3d bottom_center(0.0, 0.0, -0.5 * dimension_vel[2]);
        bottom_center = R_vel * bottom_center;
        Eigen::Vector4d location_vel_bottom_4(location_vel(0) + bottom_center(0), location_vel(1) + bottom_center(1), location_vel(2) + bottom_center(2), 1.0);
        Eigen::Vector4d location_cam_bottom_4 = Tr * location_vel_bottom_4;
        Eigen::Vector3d location_cam(location_cam_bottom_4(0), location_cam_bottom_4(1), location_cam_bottom_4(2));

        std::vector<Eigen::Vector4d> vertex_4(8);
        vertex_4.resize(8);
        for (int i=0; i<8;i++)
        {
            vertex_4[i] << vertex[i](0), vertex[i](1), vertex[i](2), 1.0;
        }
        std::vector<Eigen::Vector3d> vertex_image3(8);
        vertex_image3.resize(8);
        for (int i=0; i<8;i++)
        {
            Eigen::Vector4d location_cam_4;
            location_cam_4 = Tr * vertex_4[i];
            // vertex_image3[i] = P2 * Tr * vertex_4[i];
            Eigen::Vector2d location_cam_norm;
            location_cam_norm << location_cam_4(0)/location_cam_4(2), location_cam_4(1)/location_cam_4(2);
            double r = location_cam_norm.norm();
            Eigen::Vector2d location_cam_after_distort;
            location_cam_after_distort(0) = location_cam_norm(0) * (1 + k1*pow(r,2) + k2* pow(r,4) + k3*pow(r,6)) + 2. * p1 * location_cam_norm(0) * location_cam_norm(1) + p2*(pow(r,2) + 2*location_cam_norm(0)*location_cam_norm(0));
            location_cam_after_distort(1) = location_cam_norm(1) * (1 + k1*pow(r,2) + k2* pow(r,4) + k3*pow(r,6)) + 2. * p2 * location_cam_norm(0) * location_cam_norm(1) + p1*(pow(r,2) + 2*location_cam_norm(1)*location_cam_norm(1));
            vertex_image3[i](0) = P2(0,0) * location_cam_after_distort(0) + P2(0,2);
            vertex_image3[i](1) = P2(1,1) * location_cam_after_distort(1) + P2(1,2);
            // vertex_image3[i](0) = vertex_image3[i](0)/ abs(vertex_image3[i](2));
            // vertex_image3[i](1) = vertex_image3[i](1)/ abs(vertex_image3[i](2));
            vertex_image3[i](2) = 1.; // does not matter
        }
        double left = vertex_image3[0](0);
        double right = vertex_image3[0](0);
        double top = vertex_image3[0](1);
        double bottom = vertex_image3[0](1);
        for (int i=0;i<8;i++)
        {   
            std::cout << "x: " << vertex_image3[i](0) << ", y: " << vertex_image3[i](1) << " z:" << vertex_image3[i](2)<<std::endl;
            if (vertex_image3[i](0) < left) left = vertex_image3[i](0);
            if (vertex_image3[i](0) > right) right = vertex_image3[i](0);
            if (vertex_image3[i](1) < top) top = vertex_image3[i](1);
            if (vertex_image3[i](1) > bottom) bottom = vertex_image3[i](1);
        }
        std::cout << "left: " << left << "top, " << top << "right: " << right << "bottom: " << bottom << std::endl;
        if (((left > -0.01 && left < (image_width + 0.01)) || (right > -0.01 && right < (image_width + 0.01))) && ((top > -0.01 && top < (image_height + 0.01)) || (bottom > -0.01 && bottom < (image_height + 0.01))))
        {
            left = left > -0.01? left:0.;
            right = right < (image_width + 0.01)? right:image_width;
            top = top > -0.01? top:0.;
            bottom = bottom < (image_height + 0.01)? bottom:image_height; 
            if(!(class_vector[index].size() < 5))
            {   
                // if (class_type == "van") class_type = "car";
                std::cout << "This object is within FoV" << std::endl;
                outputfile << frame_i << " " << index << " " << class_type << " " << -1 << " " << -1 << " ";
                outputfile << alpha << " " << left << " " << top << " " << right << " " << bottom << " "; 
                outputfile << dimension_vel[2] << " " << dimension_vel[1] << " " << dimension_vel[0] << " ";
                outputfile << location_cam[0] << " " << location_cam[1] << " " << location_cam[2] << " ";
                outputfile << rotation_y << " " << -1 << std::endl;
            }
        }
    }
    outputfile.close();
}