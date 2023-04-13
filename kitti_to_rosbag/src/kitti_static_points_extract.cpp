/*
Copyright (c) 2016, Helen Oleynikova, ETH Zurich, Switzerland
You can contact the author at <helen dot oleynikova at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <rosbag/bag.h>
#include <tf/tfMessage.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <unordered_set>

#include "kitti_to_rosbag/kitti_parser.h"
#include "kitti_to_rosbag/kitti_ros_conversions.h"

namespace kitti {

class KittiBagConverter {
 public:
  KittiBagConverter(const std::string& calibration_path,
                    const std::string& dataset_path,
                    const std::string& output_filename,
                    const std::string& sequence_num,
                    const std::string& label_out_file_in);

  void convertAll();
  bool convertEntry(uint64_t entry, pcl::PointCloud<pcl::PointXYZI> &pointcloud);
  void convertTf(uint64_t timestamp_ns, const Transformation& imu_pose);
  void writelabels(bool debug, std::string filename, pcl::PointCloud<pcl::PointXYZI> pointcloud, std::vector<Eigen::Vector3d> min_points, std::vector<Eigen::Vector3d> max_points, std::vector<Eigen::Matrix3d> R, std::vector<std::string> types, Eigen::Matrix3d R_cam_vel, Eigen::Vector3d t_cam_vel);

 private:
  kitti::KittiParser parser_;

  rosbag::Bag bag_;

  std::string world_frame_id_;
  std::string imu_frame_id_;
  std::string cam_frame_id_prefix_;
  std::string velodyne_frame_id_;

  std::string pose_topic_;
  std::string transform_topic_;
  std::string pointcloud_topic_;

  std::string label_out_file;
};

KittiBagConverter::KittiBagConverter(const std::string& calibration_path,
                                     const std::string& dataset_path,
                                     const std::string& output_filename,
                                     const std::string& sequence_num,
                                     const std::string& label_out_file_in)
    : parser_(calibration_path, dataset_path, sequence_num, true),
      label_out_file(label_out_file_in),
      world_frame_id_("world"),
      imu_frame_id_("imu"),
      cam_frame_id_prefix_("cam"),
      velodyne_frame_id_("camera_init"),
      pose_topic_("pose_imu"),
      transform_topic_("transform_imu"),
      pointcloud_topic_("velodyne_points") {
  // Load all the timestamp maps and calibration parameters.
  parser_.loadCalibration();
  // parser_.loadTimestampMaps();

  bag_.open(output_filename, rosbag::bagmode::Write);
}

void KittiBagConverter::convertAll() {
  std::vector<int> frame, id;
  std::vector<Eigen::Vector3d> min_points, max_points;
  std::vector<Eigen::Matrix3d> Rs;
  std::vector<std::string> types;
  std::vector<std::vector<Eigen::Vector3d>> min_points_byframe, max_points_byframe;
  std::vector<std::vector<Eigen::Matrix3d>> R_byframe;
  std::vector<std::vector<std::string>> types_byframe;
  std::string label_folder = "label_02";
  if(parser_.getGTboudingbox(label_folder, frame, id, min_points, max_points, Rs, types))
  { 
    visualization_msgs::MarkerArray clusters;
    uint64_t timestamp_ns;
    ros::Time timestamp_ros;
    Transformation T_cam0_vel = parser_.T_cam0_vel().inverse();
    kindr::minimal::RotationQuaternionTemplate<double> Rot_cam_vel = T_cam0_vel.getRotation();
    Eigen::Quaterniond quat_cam_vel = Rot_cam_vel.toImplementation();
    Eigen::Matrix3d R_cam_vel(quat_cam_vel);
    Eigen::Vector3d t_cam_vel = T_cam0_vel.getPosition();
    std::cout << "t_cam_vel: " << t_cam_vel << std::endl;
    std::cout << "R_cam_vel: " << R_cam_vel << std::endl;

    min_points_byframe.resize(frame[frame.size() - 1] + 1);
    max_points_byframe.resize(frame[frame.size() - 1] + 1);
    R_byframe.resize(frame[frame.size() - 1] + 1);
    types_byframe.resize(frame[frame.size() - 1] + 1);

    int last_frame = 0;
    int max_id = 0;
    std::unordered_set<int> idset;
    std::cout << "frame_size: " << frame.size() << std::endl;
    for(int i = 0; i < frame.size(); i++)
    { 
      if(frame[i] != last_frame)
      { 
        timestamp_ns = ros::TIME_MIN.toNSec() + 1e8 * last_frame;
        timestampToRos(timestamp_ns, &timestamp_ros);
        for(int j = 0; j <= max_id; j++)
        {
          if(!idset.count(j))
          {
            visualization_msgs::Marker cluster;
            cluster.header.frame_id = velodyne_frame_id_;
            // cluster.header.frame_id = "aft_mapped";
            cluster.header.stamp = timestamp_ros;
            cluster.ns = "cluster_gt";
            cluster.id = j;
            cluster.type = visualization_msgs::Marker::LINE_STRIP;
            cluster.action = visualization_msgs::Marker::ADD;
            cluster.scale.x = 0.1;
            cluster.scale.y = 0.1;
            cluster.scale.z = 0.1;
            // cluster.color.r = reds[j%9];
            // cluster.color.g = greens[j%9];
            // cluster.color.b = blues[j%9];
            cluster.color.r = 1;
            cluster.color.g = 1;
            // std::cout<<"r: "<<reds[j%9]<<"  "<<"g: "<<greens[j%9]<<"  "<<"b: "<<blues[j%9]<<std::endl;
            cluster.color.a = 0;
            cluster.lifetime = ros::Duration();
            clusters.markers.push_back(cluster);
          }
        }
        bag_.write("/cluster_vis_gt", timestamp_ros,
                clusters);
        clusters.markers.clear();
        idset.clear();
        // last_frame = frame[i];
        last_frame ++;
        while (last_frame != frame[i])
        {
          timestamp_ns = ros::TIME_MIN.toNSec() + 1e8 * last_frame;
          timestampToRos(timestamp_ns, &timestamp_ros);
          for(int j = 0; j <= max_id; j++)
          {
            visualization_msgs::Marker cluster;
            cluster.header.frame_id = velodyne_frame_id_;
            // cluster.header.frame_id = "aft_mapped";
            cluster.header.stamp = timestamp_ros;
            cluster.ns = "cluster_gt";
            cluster.id = j;
            cluster.type = visualization_msgs::Marker::LINE_STRIP;
            cluster.action = visualization_msgs::Marker::ADD;
            cluster.scale.x = 0.1;
            cluster.scale.y = 0.1;
            cluster.scale.z = 0.1;
            // cluster.color.r = reds[j%9];
            // cluster.color.g = greens[j%9];
            // cluster.color.b = blues[j%9];
            if (label_folder == "label_02" || label_folder == "label_dyn")
            {
              cluster.color.r = 1;
              cluster.color.g = 1;
            }
            else
            {
              cluster.color.b = 1;
            }
            // std::cout<<"r: "<<reds[j%9]<<"  "<<"g: "<<greens[j%9]<<"  "<<"b: "<<blues[j%9]<<std::endl;
            cluster.color.a = 0;
            cluster.lifetime = ros::Duration();
            clusters.markers.push_back(cluster);
          }
          bag_.write("/cluster_vis_gt", timestamp_ros,
                  clusters);
          clusters.markers.clear();
          last_frame++;
          }
      }
      visualization_msgs::Marker cluster;
      cluster.header.frame_id = velodyne_frame_id_;
      // cluster.header.frame_id = "camera_init";
      timestamp_ns = ros::TIME_MIN.toNSec() + 1e8 * frame[i];
      timestampToRos(timestamp_ns, &timestamp_ros);
      cluster.header.stamp = timestamp_ros;
      cluster.ns = "cluster_gt";
      cluster.id = id[i];
      if(id[i] > max_id) max_id = id[i];
      idset.insert(id[i]);
      cluster.type = visualization_msgs::Marker::LINE_STRIP;
      cluster.action = visualization_msgs::Marker::ADD;
      cluster.scale.x = 0.1;
      cluster.scale.y = 0.1;
      cluster.scale.z = 0.1;
      // cluster.color.r = reds[j%9];
      // cluster.color.g = greens[j%9];
      // cluster.color.b = blues[j%9];
      if (label_folder == "label_02" || label_folder == "label_dyn")
      {
        cluster.color.r = 1;
        cluster.color.g = 1;
      }
      else
      {
        cluster.color.b = 1;
      }
      // std::cout<<"r: "<<reds[j%9]<<"  "<<"g: "<<greens[j%9]<<"  "<<"b: "<<blues[j%9]<<std::endl;
      cluster.color.a = 1;
      cluster.lifetime = ros::Duration();
      Eigen::Matrix3d R = R_cam_vel * Rs[i];
      Eigen::Vector3d min_point = R_cam_vel * min_points[i] + t_cam_vel;
      Eigen::Vector3d max_point = R_cam_vel * max_points[i] + t_cam_vel;
      draw_bbox(cluster, R.transpose() * min_point, R.transpose() * max_point, R);
      min_points_byframe[last_frame].push_back(R.transpose() * min_point);
      max_points_byframe[last_frame].push_back(R.transpose() * max_point);
      R_byframe[last_frame].push_back(R);
      types_byframe[last_frame].push_back(types[i]);
      clusters.markers.push_back(cluster);
      if(i == frame.size()-1)
      {
        timestamp_ns = ros::TIME_MIN.toNSec() + 1e8 * frame[i];
        timestampToRos(timestamp_ns, &timestamp_ros);
        for(int j = 0; j <= max_id; j++)
        {
          if(!idset.count(j))
          {
            visualization_msgs::Marker cluster;
            cluster.header.frame_id = velodyne_frame_id_;
            // cluster.header.frame_id = "aft_mapped";
            cluster.header.stamp = timestamp_ros;
            cluster.ns = "cluster_gt";
            cluster.id = j;
            cluster.type = visualization_msgs::Marker::LINE_STRIP;
            cluster.action = visualization_msgs::Marker::ADD;
            cluster.scale.x = 0.1;
            cluster.scale.y = 0.1;
            cluster.scale.z = 0.1;
            // cluster.color.r = reds[j%9];
            // cluster.color.g = greens[j%9];
            // cluster.color.b = blues[j%9];
            if (label_folder == "label_02" || label_folder == "label_dyn")
            {
              cluster.color.r = 1;
              cluster.color.g = 1;
            }
            else
            {
              cluster.color.b = 1;
            }
            // std::cout<<"r: "<<reds[j%9]<<"  "<<"g: "<<greens[j%9]<<"  "<<"b: "<<blues[j%9]<<std::endl;
            cluster.color.a = 0;
            cluster.lifetime = ros::Duration();
            clusters.markers.push_back(cluster);
          }
        }
        bag_.write("/cluster_vis_gt", timestamp_ros,
                 clusters);
        clusters.markers.clear();
        idset.clear();
        last_frame = frame[i];
      }
    }
    timestamp_ns = ros::TIME_MIN.toNSec() + 1e8 * (frame[frame.size()-1] + 1);
    timestampToRos(timestamp_ns, &timestamp_ros);
    for(int j = 0; j <= max_id; j++)
    {
      visualization_msgs::Marker cluster;
      cluster.header.frame_id = velodyne_frame_id_;
      // cluster.header.frame_id = "aft_mapped";
      cluster.header.stamp = timestamp_ros;
      cluster.ns = "cluster_gt";
      cluster.id = j;
      cluster.type = visualization_msgs::Marker::LINE_STRIP;
      cluster.action = visualization_msgs::Marker::ADD;
      cluster.scale.x = 0.1;
      cluster.scale.y = 0.1;
      cluster.scale.z = 0.1;
      // cluster.color.r = reds[j%9];
      // cluster.color.g = greens[j%9];
      // cluster.color.b = blues[j%9];
      if (label_folder == "label_02" || label_folder == "label_dyn")
      {
        cluster.color.r = 1;
        cluster.color.g = 1;
      }
      else
      {
        cluster.color.b = 1;
      }
      // std::cout<<"r: "<<reds[j%9]<<"  "<<"g: "<<greens[j%9]<<"  "<<"b: "<<blues[j%9]<<std::endl;
      cluster.color.a = 0;
      cluster.lifetime = ros::Duration();
      clusters.markers.push_back(cluster);
    }
    bag_.write("/cluster_vis_gt", timestamp_ros,
                 clusters);
  }
  uint64_t entry = 0;
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  Transformation T_cam0_vel = parser_.T_cam0_vel().inverse();
  kindr::minimal::RotationQuaternionTemplate<double> Rot_cam_vel = T_cam0_vel.getRotation();
  Eigen::Quaterniond quat_cam_vel = Rot_cam_vel.toImplementation();
  Eigen::Matrix3d R_cam_vel(quat_cam_vel);
  Eigen::Vector3d t_cam_vel = T_cam0_vel.getPosition();
  while (convertEntry(entry, pointcloud)) {
    std::cout << "entry: " << entry << std::endl;
    std::string out_file = label_out_file;
    std::stringstream s;
    s << std::setw(6) << std::setfill('0') << entry;
    out_file += s.str(); 
    out_file.append(".bin");
    if (entry <= min_points_byframe.size() - 1)
    { 
      bool debug = false;
      if (entry >438) debug = false;
      writelabels(debug, out_file, pointcloud, min_points_byframe[entry], max_points_byframe[entry], R_byframe[entry], types_byframe[entry], R_cam_vel, t_cam_vel);
    }
    else
    {
      std::vector<Eigen::Vector3d> empty_vect;
      std::vector<Eigen::Matrix3d> empty_matr;
      std::vector<std::string> empty_string;
      writelabels(false, out_file, pointcloud, empty_vect, empty_vect, empty_matr, empty_string, R_cam_vel, t_cam_vel);
    }
    
    entry++;
  }
  std::cout << "Converted " << entry << " entries into a rosbag.\n";
}

bool KittiBagConverter::convertEntry(uint64_t entry, pcl::PointCloud<pcl::PointXYZI> &pointcloud) {
  ros::Time timestamp_ros;
  uint64_t timestamp_ns;

  // Convert poses + TF transforms.
  // Transformation pose;
  // if (parser_.getPoseAtEntry(entry, &timestamp_ns, &pose)) {
  //   geometry_msgs::PoseStamped pose_msg;
  //   geometry_msgs::TransformStamped transform_msg;

  //   timestampToRos(timestamp_ns, &timestamp_ros);
  //   pose_msg.header.frame_id = world_frame_id_;
  //   pose_msg.header.stamp = timestamp_ros;
  //   transform_msg.header.frame_id = world_frame_id_;
  //   transform_msg.header.stamp = timestamp_ros;

  //   poseToRos(pose, &pose_msg);
  //   transformToRos(pose, &transform_msg);

  //   bag_.write(pose_topic_, timestamp_ros, pose_msg);
  //   bag_.write(transform_topic_, timestamp_ros, transform_msg);

  //   convertTf(timestamp_ns, pose);
  // } else {
  //   return false;
  // }

  // Convert images.
  cv::Mat image;
  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    if (parser_.getImageAtEntry(entry, cam_id, &timestamp_ns, &image)) {
      timestamp_ns += ros::TIME_MIN.toNSec();
      timestampToRos(timestamp_ns, &timestamp_ros);

      sensor_msgs::Image image_msg;
      imageToRos(image, &image_msg);
      image_msg.header.stamp = timestamp_ros;
      image_msg.header.frame_id = getCameraFrameId(cam_id);

      // TODO(helenol): cache this.
      // Get the calibration info for this camera.
      CameraCalibration cam_calib;
      parser_.getCameraCalibration(cam_id, &cam_calib);
      sensor_msgs::CameraInfo cam_info;
      calibrationToRos(cam_id, cam_calib, &cam_info);
      cam_info.header = image_msg.header;

      bag_.write(getCameraFrameId(cam_id) + "/image_raw", timestamp_ros,
                 image_msg);
      bag_.write(getCameraFrameId(cam_id) + "/camera_info", timestamp_ros,
                 cam_info);
    }
  }
  // Convert pointclouds.
  if (parser_.getPointcloudAtEntry(entry, &timestamp_ns, &pointcloud)) {
    timestamp_ns += ros::TIME_MIN.toNSec();
    timestampToRos(timestamp_ns, &timestamp_ros);

    // This value is in MICROSECONDS, not nanoseconds.
    pointcloud.header.stamp = timestamp_ns / 1000;
    pointcloud.header.frame_id = velodyne_frame_id_;
    Transformation pose;
    convertTf(timestamp_ns, pose);

    bag_.write(pointcloud_topic_, timestamp_ros, pointcloud);
  }
  else
  {
    return false;
  }

  return true;
}

void KittiBagConverter::convertTf(uint64_t timestamp_ns,
                                  const Transformation& imu_pose) {
  tf::tfMessage tf_msg;
  ros::Time timestamp_ros;
  timestampToRos(timestamp_ns, &timestamp_ros);

  // Create the full transform chain.
  Transformation T_imu_world = imu_pose;
  Transformation T_vel_imu = parser_.T_vel_imu();
  Transformation T_cam_imu;
  Transformation T_vel_cam0 = parser_.T_cam0_vel();

  geometry_msgs::TransformStamped tf_imu_world, tf_vel_imu, tf_cam_imu, tf_vel_cam0;
  transformToRos(T_imu_world, &tf_imu_world);
  tf_imu_world.header.frame_id = world_frame_id_;
  tf_imu_world.child_frame_id = imu_frame_id_;
  tf_imu_world.header.stamp = timestamp_ros;
  transformToRos(T_vel_imu.inverse(), &tf_vel_imu);
  tf_vel_imu.header.frame_id = imu_frame_id_;
  tf_vel_imu.child_frame_id = velodyne_frame_id_;
  tf_vel_imu.header.stamp = timestamp_ros;
  transformToRos(T_vel_cam0, &tf_vel_cam0);
  tf_vel_cam0.header.frame_id = "camera";
  tf_vel_cam0.child_frame_id = velodyne_frame_id_;
  tf_vel_cam0.header.stamp = timestamp_ros;

  // Put them into one tf_msg.
  // tf_msg.transforms.push_back(tf_imu_world);
  // tf_msg.transforms.push_back(tf_vel_imu);
  tf_msg.transforms.push_back(tf_vel_cam0);

  // Get all of the camera transformations as well.
  // for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
  //   T_cam_imu = parser_.T_camN_imu(cam_id);
  //   transformToRos(T_cam_imu.inverse(), &tf_cam_imu);
  //   tf_cam_imu.header.frame_id = imu_frame_id_;
  //   tf_cam_imu.child_frame_id = getCameraFrameId(cam_id);
  //   tf_cam_imu.header.stamp = timestamp_ros;
  //   tf_msg.transforms.push_back(tf_cam_imu);
  // }

  bag_.write("/tf", timestamp_ros, tf_msg);
}

void KittiBagConverter::writelabels(bool debug, std::string filename, pcl::PointCloud<pcl::PointXYZI> pointcloud, std::vector<Eigen::Vector3d> min_points, std::vector<Eigen::Vector3d> max_points, std::vector<Eigen::Matrix3d> R, std::vector<std::string> types, Eigen::Matrix3d R_cam_vel, Eigen::Vector3d t_cam_vel)
{   
  std::ofstream out;
  out.open(filename, std::ios::out  | std::ios::binary);
  bool pub = false;
  for (int p_index = 0; p_index < pointcloud.size(); p_index++)
  { 
    if(debug && p_index ==29590) pub = true;
    else pub = false;
    Eigen::Vector3d point(pointcloud[p_index].x, pointcloud[p_index].y, pointcloud[p_index].z);
    if(pub) std::cout << "point: " << point.transpose() << std::endl;
    Eigen::Vector3d point_cam = R_cam_vel.transpose() * (point - t_cam_vel);
    double horizon = atan2f(float(point_cam(0)), float(point_cam(2)));
    int bbox_size = min_points.size();
    bool insidebox = false;
    int inside_which_box = -1;
    for (int bbox_index = 0; bbox_index < bbox_size; bbox_index++)
    {
      Eigen::Vector3d point_bbox = R[bbox_index].transpose() * point;
      bool insidebox_this = false;
      if(pub) std::cout << "min_point: " << min_points[bbox_index].transpose() << " max_point: " << max_points[bbox_index].transpose() << std::endl;
      if(pub) std::cout << "point_bboxbody: " << point_bbox.transpose() << std::endl;
      // Eigen::Vector3d theroshold(0.40, 0.20, 0.40);
      // Eigen::Vector3d ground_height(0.0, 0.30, 0.0);
      Eigen::Vector3d theroshold;
      Eigen::Vector3d ground_height(0.0, 0.0, 0.0);
      for (int i = 0; i < 3; i++)
      { 
        // if(types[bbox_index] == "Pedestrian" || types[bbox_index] == "Cyclist") theroshold << 0.20, 0.20, 0.20;
        // else theroshold << 0.40, 0.20, 0.40;
        theroshold(i) = 0.5 * (max_points[bbox_index][i] - min_points[bbox_index][i]);
        if ((min_points[bbox_index][i] - theroshold[i]) < point_bbox[i] && (max_points[bbox_index][i] + theroshold[i] - ground_height[i]) > point_bbox[i]) insidebox_this = true;
        else 
        {
          insidebox_this = false;
          break;
        }
      }
      if(insidebox_this)
      {
        insidebox = true;
        inside_which_box = bbox_index;
        break;
      }
      else continue;
    }
    if(pub) std::cout << "insidebox: " << insidebox << std::endl;
    bool insidefov = true;
    if (-40./180. * 3.14159 > horizon || 40./180. * 3.14159 < horizon)
    {
      insidefov = false;
    }
    if(insidebox)
    { 
      int tmp = 251 + inside_which_box;
      if(types[inside_which_box] == "Person") tmp += 0 * 1000;
      else if(types[inside_which_box] == "Truck") tmp += 1 * 1000;
      else if(types[inside_which_box] == "Car") tmp += 2* 1000;
      else if(types[inside_which_box] == "Tram") tmp += 3* 1000;
      else if(types[inside_which_box] == "Pedestrian") tmp += 4* 1000;
      else if(types[inside_which_box] == "Cyclist") tmp += 5* 1000;
      else if(types[inside_which_box] == "Van") tmp += 6* 1000;
    //   out.write((char*)&tmp, sizeof(int));
      out.write((char*)&pointcloud[p_index].x, 3 * sizeof(float));
      out.write((char*)&pointcloud[p_index].intensity, sizeof(float));
    }
    else if (insidefov)
    {
      int tmp = 9;
    //   out.write((char*)&tmp, sizeof(int));
    }
    else
    {
      int tmp = -1;
    //   out.write((char*)&tmp, sizeof(int));
    }
  }
  out.close();
}
}  // namespace kitti

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  if (argc < 4) {
    std::cout << "Usage: rosrun kitti_to_rosbag kitti_rosbag_converter "
                 "calibration_path dataset_path output_path\n";
    std::cout << "Note: no trailing slashes.\n";
    return 0;
  }

  const std::string calibration_path = argv[1];
  const std::string dataset_path = argv[2];
  const std::string output_path = argv[3];
  const std::string label_out_path = argv[4];
  const std::string sequence_num = argv[5];
  std::cout << ros::TIME_MIN.toSec() << std::endl;
  std::cout << ros::TIME_MIN.toNSec() << std::endl;
  kitti::KittiBagConverter converter(calibration_path, dataset_path,
                                     output_path, sequence_num, label_out_path);
  converter.convertAll();

  return 0;
}
