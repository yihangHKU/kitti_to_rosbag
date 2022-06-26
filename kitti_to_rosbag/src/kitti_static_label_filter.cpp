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
    int frame{-1};
    int track_id{-1};
    std::string track_class;
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
    std::vector<std::vector<object_s>> tracks(200);

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
            std::cout << "projection_mat: " <<projection_mat << std::endl;
            Eigen::Matrix3d R = projection_mat.block<3,3>(0,0);
            Eigen::Vector3d t = projection_mat.block<3,1>(0,3);
            Transformation T_vel_cam;
            T_vel_cam.getRotation() = Rotation::fromApproximateRotationMatrix(R);
            std::cout << "R: " << R << std::endl;
            T_vel_cam.getPosition() = t;
            T_cam_vel_ = T_vel_cam.inverse();
        }
    }

    // read label data
    while (std::getline(labelfile, line)){
        std::stringstream line_stream(line);
        std::vector<double> parsed_doubles;
        object_s object;

        while (!line_stream.eof()) {
            std::string element;
            std::getline(line_stream, element, ' ');
            if (element.empty()) {
                continue;
            }
            try {
                if(element.compare("Car") == 0) object.track_class="Car";
                if(element.compare("Pedestrian") == 0) object.track_class="Pedestrian";
                if(element.compare("Van") == 0) object.track_class="Van";
                if(element.compare("Truck") == 0) object.track_class="Truck";
                if(element.compare("Cyclist") == 0) object.track_class="Cyclist";
                if(element.compare("Person") == 0) object.track_class="Person";
                parsed_doubles.emplace_back(std::stod(element));
            } catch (const std::exception& exception) {
            // std::cout << "Could not parse number in import file.\n";
            }
        }
        Eigen::Matrix<double, 16, 1> V(parsed_doubles.data());
        if (V[1] != -1)
        {   
            object.frame = V[0];
            object.track_id = V[1];
            Eigen::Vector3d location(V[12], V[13], V[14]);
            // Eigen::Vector3d location(V[14], V[13], V[12]);
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
            // if (object.frame <= 5)
            // {   
            //     std::cout << "track id: " << object.track_id << std::endl;
            //     std::cout << "velodyne to globle t: " << t_vel_gl.transpose() << std::endl;
            //     std::cout << "location_velodyne frame: " << object.location_vel.transpose() << std::endl;
            //     std::cout << "location_global frame: " << object.location_gl.transpose() << std::endl;
            // }
            if(tracks[object.track_id].size() == 0)
            {
                tracks[object.track_id].resize(2000);
            }
            tracks[object.track_id][object.frame] = object;
        }
    }
    kindr::minimal::RotationQuaternionTemplate<double> Rot_cam_vel = T_cam_vel_.getRotation();
    Eigen::Quaterniond quat_cam_vel = Rot_cam_vel.toImplementation();
    Eigen::Matrix3d R_cam_vel(quat_cam_vel);
    std::cout << "R_cam_vel: " << R_cam_vel << std::endl;
    for(int i = 0; i < tracks.size(); i++)
    {
        bool track_static = false;
        double thresold = 0.10;
        if (tracks[i].size() <= 1) 
        {   
            track_static = true;
            for (int j = 0; j < tracks[i].size(); j++)
            {
                tracks[i][j].frame = -1;
            }
            continue;
        }
        for (int j = 0; j < tracks[i].size() - 1; j++)
        {   
            if(tracks[i][j].track_class == "Pedestrian" || tracks[i][j].track_class == "Cyclist")
            {
                thresold =0.05;
            }
            if(tracks[i][j].frame >-1 && tracks[i][j+1].frame >-1)
            {
                Eigen::Vector3d curr_loc = tracks[i][j].location_gl;
                Eigen::Vector3d next_loc = tracks[i][j+1].location_gl;
                if(i==-1)
                {   
                    // std::cout << "current loc: " << curr_loc.transpose() << std::endl;
                    // std::cout << "next loc: " << next_loc.transpose() << std::endl;
                    std::cout << "frame: " << j << std::endl;
                    std::cout << "location diff of track 3: " <<  (curr_loc - next_loc).norm() << std::endl;
                }
                // if ((curr_loc - next_loc).norm() < thresold || j <= 2 || i==5 || i==7 || i==43 || i==59 || i==62 || i==61 || i==3 || i==4 ||i==2||i==6||i==11 ||i==18||i==22|| i==16 ||i==19|| i==23 || i==26 ||i==28 ||i==32||i==30||i==34||i==33||i==35 ||i==44 ||i==45 ||i==51||i==54) //07
                // if ((curr_loc - next_loc).norm() < thresold || j <= 2 ||i==7 ||i==15||i==16) // 08
                // if ((curr_loc - next_loc).norm() < thresold || j <= 2 || (i<= 13&& i>=10) || i<=8 || (i<=30&&i>=27) || (i<=43&&i>=32) || i==56 ||i==58||i==16||i==26||i==18||(i<=50&&i>=44)||i==59 ||i==61||i==55||i==62 ||i==64||(i<=78&&i>=71) ||(i<=85&&i>=80)) //09
                // if ((curr_loc - next_loc).norm() < thresold || j <= 2 ||i==10||i==9||i==11||i==12||i==14||i==33||i==26||i==39||i==55||i==29||i==43||i==40||i==54||i==25) // 11
                // if ((curr_loc - next_loc).norm() < thresold || j <= 2 || i==67 ||i==22 ||i==3 ||i==8||i==12 ||i==23||i==29 ||i==17 ||i==38 ||i==38||i==61 ||i==40 ||i==63||i==50||i==48) //13
                // if (tracks[i][j].track_class == "Person" || (curr_loc - next_loc).norm() < thresold || j <= 2 ||i==3||i==4||i==5||i==8||i==9||i==10||i==13) //14
                // if (tracks[i][j].track_class == "Person" || (curr_loc - next_loc).norm() < thresold || j <= 2 ||i==1||i==16||i==17 ||i==2) //15
                // if (tracks[i][j].track_class == "Person" || (curr_loc - next_loc).norm() < thresold || j <= 2 || i==65 ||i==57 ||i==12 ||i==42 ||i==67) //19
                if (tracks[i][j].track_class == "Person" || (curr_loc - next_loc).norm() < thresold || j <= 2)
                { 
                    // if(i==63) // 24 for bag 16
                    // {
                    //    track_static = false; 
                    // }
                    // else
                    // {
                        track_static = true;
                        tracks[i][j].frame = -1;
                    // }
                    // break;
                }
            }
            else if(tracks[i][j].frame >-1 && tracks[i][j+1].frame < 0 && tracks[i][j-1].frame >-1)
            {
                Eigen::Vector3d curr_loc = tracks[i][j].location_gl;
                Eigen::Vector3d last_loc = tracks[i][j-1].location_gl;
                // if(i == 0)
                // {
                //     std::cout << "location diff of track 0: " <<  (curr_loc - next_loc).norm() << std::endl;
                // }
                if ((curr_loc - last_loc).norm() < thresold)
                {
                    track_static = true;
                    tracks[i][j].frame = -1;
                    // break;
                }
            }
            else if(tracks[i][j].frame >-1 && tracks[i][j+1].frame < 0 && tracks[i][j-1].frame < 0)
            {
                tracks[i][j].frame = -1;
            }
        }
        // if(track_static)
        // {
        //     tracks[i].clear();
        //     std::cout << "The track " << i << " is recognized as static track thus been removed" << std::endl;
        // }
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
        if (V[1] != -1 && !tracks[V[1]].empty() && tracks[V[1]][V[0]].frame > -1)
        {   
            std::stringstream line_stream_d(line);
            std::string element_d;
            std::getline(line_stream_d, element_d);
            dyn_labelfile << element_d << std::endl;
        }
    }
}