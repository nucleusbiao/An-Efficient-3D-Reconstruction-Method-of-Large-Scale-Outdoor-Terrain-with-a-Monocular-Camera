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
// 鍙傛暟鏂囦欢涓庡瓧鍏告枃浠�1锟�71锟�1锟�771锟�1锟�71锟�1锟�777
// 濡傛灉浣犵郴缁熶笂鐨勮矾寰勪笉鍚岋紝璇蜂慨鏀瑰畠
string parameterFile = "/home/zjd/orb_to_depth/src/orbslam/Examples/Monocular/ip640_undistoried.yaml";
//string parameterFile = "./usbost.yaml";
string vocFile = "/home/zjd/orb_to_depth/src/orbslam/Vocabulary/ORBvoc.bin";
// 瑙嗛鏂囦欢
string videoFile = "/home/zjd/shujuji/ip24/rgb.mp4";
int main(int argc, char **argv) {
    ros::init(argc, argv, "tozoe");
    ros::start();
    

    // 鍒涘缓鍥惧儚璇锋眰鏈嶅姟瀹㈡埛
    // ros::ServiceClient client = nh.serviceClient<orbslam::depthimage>("depthimage");
    // 澹版槑 ORB-SLAM2 绯荤粺
    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true);
    // 鑾峰彇瑙嗛鍥惧儚
    cv::VideoCapture cap(videoFile);    // change to 0 if you want to use USB camera.
    // 璁板綍绯荤粺鏃堕棿
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
