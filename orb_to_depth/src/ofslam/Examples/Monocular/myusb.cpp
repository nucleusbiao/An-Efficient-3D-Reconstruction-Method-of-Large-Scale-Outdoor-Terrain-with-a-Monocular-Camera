#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "System.h"
#include <string>
#include <chrono>   // for time stamp
#include <iostream>

#include "ros/ros.h"
#include "zoetodepth/depthimage.h"
#include <cv_bridge/cv_bridge.h>

using namespace std;
// 如果你系统上的路径不同，请修改它
string parameterFile = "/home/zjd/orb_to_depth/src/orbslam/Examples/Monocular/usbost.yaml";
//string parameterFile = "./usbost.yaml";
string vocFile = "/home/zjd/orb_to_depth/src/orbslam/Vocabulary/ORBvoc.bin";
// 视频文件
int main(int argc, char **argv) {
    ros::init(argc, argv, "tozoe");
    ros::start();
    

    // 创建图像请求服务客户
    // ros::ServiceClient client = nh.serviceClient<orbslam::depthimage>("depthimage");
    // 声明 ORB-SLAM2 系统
    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true);
    // 获取视频图像
    cv::VideoCapture cap(0);    // change to 0 if you want to use USB camera.
    // 分辨率设为640x480
    // cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);//设置采集视频的宽度
    // cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);//设置采集视频的高度
    
    // 记录系统时间
    auto start = chrono::system_clock::now();

    while (1) {
        cv::Mat frame;
        cap >> frame;   // 读取相机数据
        cv::resize(frame, frame, cv::Size(640, 480));
        
        if ( frame.data == nullptr )
            continue;
        // rescale because image is too large
        //cv::Mat frame_resized;
        //cv::resize(frame, frame_resized, cv::Size(640,480));
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        // sensor_msgs::Image msg1 = *msg;
        // orbslam::depthimage srv;
        // srv.request.rgbimage = *msg;
        // client.call(srv);
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.depthimage, sensor_msgs::image_encodings::TYPE_32FC1);

        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackMonocular(frame, double(timestamp.count())/1000.0);
        cv::waitKey(30);
    }
    return 0;


}
