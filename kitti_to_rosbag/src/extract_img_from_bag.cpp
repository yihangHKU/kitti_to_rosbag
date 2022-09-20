#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
using namespace cv; 
using namespace std;

// hello realsense 

int maxCorners = 20;
int maxblobColor = 1;
int blobColor = 1;
int minCircularity = 8;
int maxCircularity = 10;
int blobarea = 10;
int maxblobarea = 100;
int minConvexity = 85;
int maxConvexity = 100;
int minInertiaRatio = 30;
int maxInertiaRatio = 100;
int radius = 2;
RNG rng(12345);
const char* source_window = "Source image";
const char* corners_window = "Corners detected";
const char* blob_window = "blob image";
static const std::string OPENCV_WINDOW = "Image window";
void goodFeaturesToTrack_Demo( int, void* );
Mat gray_(Size(848, 480), CV_8UC1);
Mat color(Size(848, 480), CV_8UC3);
Mat color_(Size(848, 480), CV_8UC3);
Mat gray(Size(848, 480), CV_8UC1);
deque<Mat> color_deque;
deque<Mat> gray_deque;
cv_bridge::CvImagePtr cv_ptr;
vector<Point2f> corners_;
float centroid_[3] = {0, 0, 0};
vector<float> direction_ = {0, 0 , 0};
vector<KeyPoint> last_kp;
bool detect_init = false;
bool write_jpg = true;
bool target_update = false;
bool obstacle_update = false;
string jpg_dir;
int counting = 0;

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
    sprintf(temp, "%d", (int)counting);
    string log_file = jpg_dir + string(temp) + ".jpg";
    imwrite(log_file, color);
    counting++;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "write_jpg");
    ros::NodeHandle nh;

    jpg_dir = argv[1];
    std::cout << jpg_dir << std::endl;
    String sub_topic = argv[2];
    ros::Subscriber color_sub = nh.subscribe(sub_topic, 1, color_cb);
    ros::Publisher color_pub = nh.advertise<sensor_msgs::Image>
            ("camera/blob_detect", 10);
    vector<Vec3f> circles;
    ros::Rate rate(1000.0);

    // for (int i = 0; i < 10; i++)
    // {
    //     ros::spinOnce();
    // }
    while(ros::ok() && write_jpg)
    {
        ros::spinOnce();
        // imshow(OPENCV_WINDOW, color);
        // blob_detect(0, 0);
        // imshow(OPENCV_WINDOW, gray_);
        // waitKey();
        std_msgs::Header header;
        cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, color_);
        sensor_msgs::Image img_msg;
        img_bridge.toImageMsg(img_msg);
        color_pub.publish(img_msg);
        rate.sleep();
    }

    
    return 0;
}