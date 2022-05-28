# kitti_to_rosbag
Dataset tools for working with the KITTI dataset raw data ( http://www.cvlibs.net/datasets/kitti/raw_data.php ) and converting it to a ROS bag. Also allows a library for direct access to poses, velodyne scans, and images. 

## Rosbag converter usage example
```
rosrun kitti_to_rosbag kitti_rosbag_converter calibration_path dataset_path output_path sequence_num
```
for example:

```
rosrun kitti_to_rosbag kitti_rosbag_converter /media/yihang/LYH/kitti_tracking/data_tracking_calib/training/calib /media/yihang/LYH/kitti_tracking/data_tracking_dataset/training /media/yihang/LYH/kitti_tracking/data_tracking_dataset/training/bag/0000.bag 0000
```
Use the generated bag to run fastlio and log the sequence_num.txt and put it in dataset_path/pose_lio for next static tracking remove, then

```
rosrun kitti_to_rosbag kitti_static_filter dataset_path calibration_path sequence_num
```

for example:

```
rosrun kitti_to_rosbag kitti_static_filter  /media/yihang/LYH/kitti_tracking/data_tracking_dataset/training /media/yihang/LYH/kitti_tracking/data_tracking_calib/training/calib 0000
```
Then generate a label_dyn file which only contains dynamic trackings
```