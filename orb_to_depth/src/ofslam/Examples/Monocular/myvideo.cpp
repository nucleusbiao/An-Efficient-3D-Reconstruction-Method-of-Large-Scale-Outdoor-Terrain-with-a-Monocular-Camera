#include <opencv2/opencv.hpp>
#include "System.h"
#include <string>
#include <chrono>   // for time stamp
#include <iostream>
#include <vector>
#include<algorithm>
#include<fstream>
#include "pointcloudmapping.h"
#include<stdlib.h>
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include <cstdio>

#include "ros/ros.h"
#include "zoetodepth/depthimage.h"
#include <cv_bridge/cv_bridge.h>

using namespace std;
// 参数文件与字典文仄1�71ￄ1�771ￄ1�71ￄ1�7771ￄ1�71ￄ1�771ￄ1�71ￄ1�7777
// 如果你系统上的路径不同，请修改它
string parameterFile = "/home/zjd/orb_to_depth/src/orbslam/Examples/Monocular/ip640_undistoried.yaml";
//string parameterFile = "./usbost.yaml";
string vocFile = "/home/zjd/orb_to_depth/src/orbslam/Vocabulary/ORBvoc.bin";
// 视频文件
string videoFile = "/home/zjd/shujuji/ip24/rgb.mp4";
int main(int argc, char **argv) {
    ros::init(argc, argv, "tozoe");
    ros::start();
    

    // 创建图像请求服务客户
    // ros::ServiceClient client = nh.serviceClient<orbslam::depthimage>("depthimage");
    // 声明 ORB-SLAM2 系统
    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true);
    // 获取视频图像
    cv::VideoCapture cap(videoFile);    // change to 0 if you want to use USB camera.
    // 记录系统时间
    auto start = chrono::system_clock::now();

    vector<float> vTimesTrack;
    int ni = 0;
    cv::Mat frame;
    cv::Mat frame_resized;
    
    while (cap.read(frame)) {

        if ( frame.data == nullptr )
            break;
        // rescale because image is too large
        cv::resize(frame, frame_resized, cv::Size(640,480));
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        // sensor_msgs::Image msg1 = *msg;
        // orbslam::depthimage srv;
        // srv.request.rgbimage = *msg;
        // client.call(srv);
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.depthimage, sensor_msgs::image_encodings::TYPE_32FC1);

        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        SLAM.TrackMonocular(frame_resized, double(timestamp.count())/1000.0);
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        // vTimesTrack[ni]=ttrack;
        vTimesTrack.push_back(ttrack);
        ni++;

        // cv::waitKey(30);
    }

    // while(SLAM.mpPointCloudMapping->loopbusy || SLAM.mpPointCloudMapping->cloudbusy)
    // {
    //     cout<<"";
    // }
    // Tracking time statistics
    while(SLAM.mpPointCloudMapping->loopbusy || SLAM.mpPointCloudMapping->cloudbusy)
    {
        cout<<"";
    }
    SLAM.mpPointCloudMapping->bStop = true;
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int n=0; n<vTimesTrack.size(); n++){
        totaltime+=vTimesTrack[n];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size()/2] << endl;
    cout << "mean tracking time: " << totaltime/vTimesTrack.size() << endl;
    //
    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveMap("/home/zjd/shujuji/ip3/sfm.txt",frame_resized.size);
    // Stop all threads
 
    
    SLAM.save();

    SLAM.Shutdown();
    return 0;


}
