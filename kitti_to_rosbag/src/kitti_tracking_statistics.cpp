#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include "kitti_to_rosbag/kitti_parser.h"
#include "kitti_to_rosbag/kitti_common.h"
#include "kitti_to_rosbag/kitti_ros_conversions.h"

using namespace std;
int main(int argc, char** argv) {
    unordered_map<string, int> class_set;
    std::vector<std::vector<Eigen::Vector3d>> total_lwh;
    for(int sequence = 0; sequence < 21; sequence++)
    {
        std::string s = std::to_string(sequence);
        string input_file;
        if (sequence < 10) input_file = "/media/yihang/LYH/kitti_tracking/data_tracking_dataset/training/label_02/000" + s + ".txt";
        else input_file= "/media/yihang/LYH/kitti_tracking/data_tracking_dataset/training/label_02/00" + s + ".txt";
        std::ifstream inputfile(input_file, std::ios::in);
        unordered_set<int> id_set;
        std::string line;
        while (std::getline(inputfile, line)) {
            std::stringstream line_stream(line);
            std::string frame;
            std::getline(line_stream, frame, ' ');
            std::string track_id;
            std::getline(line_stream, track_id, ' ');
            int frame_i = std::stoi(frame);
            int id = std::stoi(track_id);
            std::string class_type;
            std::getline(line_stream, class_type, ' ');
            // std::cout << "frame: " << frame_i << std::endl;
            // std::cout << "track_id: " << id << std::endl;
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
            Eigen::Matrix<double, 14, 1> V(parsed_doubles.data());
            Eigen::Vector3d dimension(V[9], V[8], V[7]);
            if (id != -1 && !id_set.count(id))
            {   
                id_set.insert(id);
                if(!class_set.count(class_type))
                {
                    class_set.insert({class_type, total_lwh.size()});
                    std::vector<Eigen::Vector3d> new_class_lwh;
                    new_class_lwh.push_back(dimension);
                    total_lwh.push_back(new_class_lwh);
                }
                else
                {
                    total_lwh[class_set[class_type]].push_back(dimension);
                }
            }
        }
    }
    for (auto itr = class_set.begin(); itr!=class_set.end(); ++itr)
    {
        cout << itr->first << " " <<  itr->second << " size:" << total_lwh[itr->second].size() << endl;
        int n = total_lwh[itr->second].size();
        double total_l = 0.0;
        double total_w = 0.0;
        double total_h = 0.0;
        for (int i = 0; i < n; i++)
        {   
            if (itr->first == "Truck") cout << total_lwh[itr->second][i](0) << " " << total_lwh[itr->second][i](1) << " " << total_lwh[itr->second][i](2)<<endl;
            total_l += total_lwh[itr->second][i](0);
            total_w += total_lwh[itr->second][i](1);
            total_h += total_lwh[itr->second][i](2);
        }
        double average_l = total_l/n;
        double average_w = total_w/n;
        double average_h = total_h/n;
        cout << "ave_L W H: " << average_l << " " << average_w << " " << average_h << endl;
        double total_minus_l = 0.0;
        double total_minus_w = 0.0;
        double total_minus_h = 0.0;
        for (int i = 0; i < n; i++)
        {
            total_minus_l += pow((total_lwh[itr->second][i](0) - average_l),2);
            total_minus_w += pow((total_lwh[itr->second][i](1) - average_w),2);
            total_minus_h += pow((total_lwh[itr->second][i](2) - average_h),2);
        }
        double sigma_l = sqrt(total_minus_l/n);
        double sigma_w = sqrt(total_minus_w/n);
        double sigma_h = sqrt(total_minus_h/n);
        cout << "sigma_L W H: " << sigma_l << " " << sigma_w << " " << sigma_h << endl;
    }


    

}