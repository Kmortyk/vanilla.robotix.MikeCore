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

int frame_width = 640;
int frame_height = 480;

const int max_frames = 1000;
const int frame_rate = 20;

int cur_frame = -1;

cv::VideoWriter* writer = nullptr;
std::string cur_file_name = "";

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
    cur_file_name = file_name();
    return new cv::VideoWriter(cur_file_name, cv::VideoWriter::fourcc('D','I','V','X'), 10, cv::Size(frame_width, frame_height));
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

   frame_width = msg->width;
   frame_height = msg->height;

   if(cur_frame == -1 || cur_frame > max_frames) {
       if(cur_frame > 0)
           ROS_INFO("writing video on the disk [%s]", cur_file_name.c_str());
       reset();
   }

   cur_frame++;

   writer->write(frame);
   ROS_INFO("collecting images (%d/%d)", cur_frame, max_frames);
}

int main(int argc, char **argv)
{
    create_dir();

    ros::init(argc, argv, "mike_hard_drive");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("mike_camera/raw", 1000, save_frame);
    ros::spin();
}
