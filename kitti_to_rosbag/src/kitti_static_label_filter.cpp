#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include "kitti_to_rosbag/kitti_parser.h"
#include "kitti_to_rosbag/kitti_common.h"
#include "kitti_to_rosbag/kitti_ros_conversions.h"

using namespace kitti;

struct object_s{
    int frame;
    int track_id{-1};
    Eigen::Vector3d location_cam;
    Eigen::Vector3d location_vel;
    Eigen::Vector3d location_gl;
};

int main(int argc, char** argv) {
    std::string root_dir = argv[1];
    std::string cal_dir = argv[2];
    std::string sequence_num = argv[3];
    std::cout << root_dir << std::endl;
    std::cout << sequence_num << std::endl;
    std::string label_file = root_dir + "/label_02/" + sequence_num + ".txt";
    std::string gt_file = root_dir + "/pose_lio/" + sequence_num + ".txt";
    // std::string cal_file = "/media/yihang/LYH/kitti_tracking/data_tracking_calib/training/calib/" + sequence_num + ".txt";
    std::string cal_file = cal_dir + sequence_num + ".txt";
    std::ifstream posefile(gt_file, std::ios::in);
    std::ifstream labelfile(label_file, std::ios::in);
    std::ifstream calfile(cal_file, std::ios::in);
    std::ofstream dyn_labelfile;
    dyn_labelfile.open(root_dir + "/label_dyn/" + sequence_num + ".txt", std::ios::out);
    std::vector<geometry_msgs::Pose> pose_gt;
    Transformation T_cam_vel_;
    std::vector<std::vector<object_s>> tracks(100);

    // read ground truth pose obtained from lio
    std::string line;
    while (std::getline(posefile, line)) {
        std::stringstream line_stream(line);
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
        Eigen::Matrix<double, 9, 1> V(parsed_doubles.data());
        geometry_msgs::Pose pose;
        pose.position.x = V[2];
        pose.position.y = V[3];
        pose.position.z = V[4];
        pose.orientation.x = V[5];
        pose.orientation.y = V[6];
        pose.orientation.z = V[7];
        pose.orientation.w = V[8];
        pose_gt.push_back(pose);
    }

    // read calibration data
    while (std::getline(calfile, line)) {
        std::stringstream line_stream(line);
        std::string header;
        std::getline(line_stream, header, ':');
        std::string data;
        std::getline(line_stream, data, ':');
        std::vector<double> parsed_doubles;

        if (header.compare(0, 11, "Tr_velo_cam") == 0) {
            std::stringstream line_stream_d(data);
            while (!line_stream_d.eof()) {
                std::string element;
                std::getline(line_stream_d, element, ' ');
                if (element.empty()) {
                    continue;
                }
                try {
                    parsed_doubles.emplace_back(std::stod(element));
                } catch (const std::exception& exception) {
                std::cout << "Could not parse number in import file.\n";
                }
            }
            Eigen::Matrix<double, 3, 4> projection_mat = Eigen::Matrix<double, 4, 3>(parsed_doubles.data()).transpose();
            Eigen::Matrix3d R = projection_mat.block<3,3>(0,0);
            Eigen::Vector3d t = projection_mat.block<3,1>(0,3);
            Transformation T_vel_cam;
            T_vel_cam.getRotation() = Rotation::fromApproximateRotationMatrix(R);
            T_vel_cam.getPosition() = t;
            T_cam_vel_ = T_vel_cam.inverse();
        }
    }


    // read label data
    while (std::getline(labelfile, line)){
        std::stringstream line_stream(line);
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
            // std::cout << "Could not parse number in import file.\n";
            }
        }
        Eigen::Matrix<double, 16, 1> V(parsed_doubles.data());
        if (V[1] != -1)
        {
            object_s object;
            object.frame = V[0];
            object.track_id = V[1];
            Eigen::Vector3d location(V[12], V[13], V[14]);
            object.location_cam = location;
            kindr::minimal::RotationQuaternionTemplate<double> Rot_cam_vel = T_cam_vel_.getRotation();
            Eigen::Quaterniond quat_cam_vel = Rot_cam_vel.toImplementation();
            Eigen::Matrix3d R_cam_vel(quat_cam_vel);
            Eigen::Vector3d t_cam_vel = T_cam_vel_.getPosition();
            object.location_vel = R_cam_vel * object.location_cam + t_cam_vel;
            Eigen::Vector3d t_vel_gl(pose_gt[object.frame].position.x, pose_gt[object.frame].position.y, pose_gt[object.frame].position.z);
            Eigen::Quaterniond q_vel_gl(pose_gt[object.frame].orientation.w, pose_gt[object.frame].orientation.x, pose_gt[object.frame].orientation.y, pose_gt[object.frame].orientation.z);
            Eigen::Matrix3d R_vel_gl(q_vel_gl);
            object.location_gl = R_vel_gl * object.location_vel + t_vel_gl;
            tracks[object.track_id].push_back(object);
        }
    }

    float thresold = 0.04;
    for(int i = 0; i < tracks.size(); i++)
    {
        bool track_static = false;
        if (tracks[i].size() <= 1) continue;
        for (int j = 0; j < tracks[i].size() - 1; j++)
        {
            Eigen::Vector3d curr_loc = tracks[i][j].location_gl;
            Eigen::Vector3d next_loc = tracks[i][j+1].location_gl;
            // if(i == 0)
            // {
            //     std::cout << "location diff of track 0: " <<  (curr_loc - next_loc).norm() << std::endl;
            // }
            if ((curr_loc - next_loc).norm() < thresold)
            {
                track_static = true;
                break;
            }
        }
        if(track_static)
        {
            tracks[i].clear();
            std::cout << "The track " << i << " is recognized as static track thus been removed" << std::endl;
        }
    }

    // write dynamic label file
    labelfile.clear();
    labelfile.seekg(0, std::ios::beg);
    while (std::getline(labelfile, line)){
        std::stringstream line_stream(line);
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
            // std::cout << "Could not parse number in import file.\n";
            }
        }
        Eigen::Matrix<double, 14, 1> V(parsed_doubles.data());
        if (V[1] != -1 && !tracks[V[1]].empty())
        {   
            std::stringstream line_stream_d(line);
            std::string element_d;
            std::getline(line_stream_d, element_d);
            dyn_labelfile << element_d << std::endl;
        }
    }
}