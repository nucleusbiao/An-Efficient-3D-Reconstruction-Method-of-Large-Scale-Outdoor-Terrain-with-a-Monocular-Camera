#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "zoetodepth/depthimage.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_image_client");
  ros::NodeHandle nh;

  // 创建图像请求服务客户端
  ros::ServiceClient client = nh.serviceClient<zoetodepth::depthimage>("depthimage");

  // 创建 USB 摄像头对象
  cv::VideoCapture cap(0);
  if (!cap.isOpened())
  {
    ROS_ERROR("Failed to open USB camera");
    return -1;
  }

  int i = 0;
  while (ros::ok())
  {
    // 读取图像帧
    cv::Mat frame;
    cap.read(frame);

    // 将图像帧转换为 ROS Image 格式
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    sensor_msgs::Image msg1 = *msg;
    
    zoetodepth::depthimage srv;
    srv.request.rgbimage = *msg;
    
    // 发送请求
    if (client.call(srv))
    {
      // 将返回的图像显示出来
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(srv.response.depthimage, sensor_msgs::image_encodings::TYPE_32FC1);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("777");
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return -1;
      }

      cv::imwrite("/home/gold/shujuji/111/" + std::to_string(i) + ".png", cv_ptr->image);
      cv::waitKey(1);
    }
    else
    {
      ROS_ERROR("Failed to call depthimage service");
    }

    // 等待 3 秒
    i++;
    ros::Duration(3.0).sleep();
  }

  return 0;
}