#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include "kitti_to_rosbag/kitti_parser.h"
#include "kitti_to_rosbag/kitti_common.h"
#include "kitti_to_rosbag/kitti_ros_conversions.h"
using namespace std;
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

int main(int argc, char** argv) {
    std::string sequence_num = argv[2];
    std::string input_file = argv[1] + sequence_num + ".txt";
    std::string output_file = argv[1] + sequence_num + "_gt.txt";
    std::ifstream inputfile(input_file, std::ios::in);
    std::ofstream outputfile(output_file, std::ios::out);  //读取  ;

    std::vector<std::vector<track_label_s>> track_vector_by_frame(5000); 
    std::string line;
    while (std::getline(inputfile, line)) 
    {
        std::stringstream line_stream(line);
        std::string frame;
        std::getline(line_stream, frame, ' ');
        std::string track_id;
        std::getline(line_stream, track_id, ' ');
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
        Eigen::Matrix<double, 8, 1> V(parsed_doubles.data());
        int class_type_i = V[4];
        std::string class_type;
        switch (class_type_i)
        {
            case 0:
                class_type = "pedestrian";
                break;
            case 1:
                class_type = "cyclist";
                break;
            case 2:
                class_type = "car";
                break;
            case 3:
                class_type = "motorcycle";
                break;
            case 4:
                class_type = "airplane";
                break;
            case 5:
                class_type = "bus";
                break;
            case 6:
                class_type = "train";
                break;
            case 7:
                class_type = "truck";
                break;
            default:
                break;
        }
        track_label_s track;
        track.frame_id = frame_i;
        track.track_id = index;
        track.type = class_type;
        track.truncated = -1;
        track.occluded = -1;
        track.alpha = 0.0;
        track.pixel <<V[0],V[1],V[0] + V[2],V[1] + V[3];
        track.dimension<< 0.,0.,0.;
        track.location<<0.,0.,0.;
        track.rotation = 0.0;
        track.score = -1;
        track_vector_by_frame[frame_i].push_back(track);
    }
    
    for(int i = 0; i < track_vector_by_frame.size();i++)
    {
        if(track_vector_by_frame[i].size() > 0 && i > 10)
        {
            for(int j=0; j < track_vector_by_frame[i].size();j++)
            {
                track_label_s track = track_vector_by_frame[i][j];
                outputfile << track.frame_id << ' ' << track.track_id << ' ' << "car" << ' ' << track.truncated << ' ';
                outputfile << track.occluded << ' ' << track.alpha << ' ' << track.pixel(0) << ' ' << track.pixel(1) << ' ';
                outputfile << track.pixel(2) << ' ' << track.pixel(3) << ' ' << track.dimension(0) << ' ' << track.dimension(1) << ' ';
                outputfile << track.dimension(2) << ' ' << track.location(0) << ' ' << track.location(1) << ' ' << track.location(2) << ' ';
                outputfile << track.rotation << ' ' << track.score << std::endl;
            }
            
        }
    } 
    outputfile.close();
}
