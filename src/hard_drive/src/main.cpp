#include "ros/ros.h"
#include "std_msgs/String.h"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <ctime>
#include <sys/stat.h>

const char* path = "/media/B6CE9388CE934013/video/";
// const char* path = "/mnt/sda1/Projects/Mike/video/";

const char* base_name = "mike-video-";
const char* extension = ".avi";

const int frame_width = 640;
const int frame_height = 480;
const int max_frames = 300;
const int frame_rate = 20;

int cur_frame = 0;

cv::VideoWriter* writer = nullptr;

void create_dir()
{
  if (mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1) {
    if(errno == EEXIST) {
       // alredy exists
    } else {
       // something else
        std::cout << "cannot create video directory:" << strerror(errno) << std::endl;
        exit(1);
    }
  }
}

std::string datetime()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,sizeof(buffer),"%d-%m-%Y-%H-%M-%S",timeinfo);
  std::string str(buffer);
  return str;
}

std::string file_name()
{
   std::stringstream ss;
   ss << path << base_name << datetime() << extension;
   return ss.str();
}

cv::VideoWriter* video_writer()
{
    return new cv::VideoWriter(file_name(), cv::VideoWriter::fourcc('D','I','V','X'), 10, cv::Size(frame_width, frame_height));
}

void reset()
{
    cur_frame = 0;
    if(writer != nullptr)
        writer->release();

    writer = video_writer();
}

void save_frame(const sensor_msgs::ImageConstPtr& msg)
{
   cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

   cur_frame++;
   if(cur_frame > max_frames) {
       reset();
   }

   writer->write(frame);
   ROS_INFO("Get image [%s]", file_name().c_str());
}

int main(int argc, char **argv)
{
    create_dir();
    reset();

    ros::init(argc, argv, "mike_hard_drive");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("mike_camera/raw", 1000, save_frame);
    ros::spin();
}
