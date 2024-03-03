/**
* This code is the tracking module of the visual odometer, used to generate camera pose and map points
*/


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"
#include<unistd.h>

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>

int idk = 1;
using namespace std;
// 程序中变量名的第丢�个字母如果为"m"则表示为类中的成员变量，member
// 第一个��第二个字母:
// "p"表示指针数据类型
// "n"表示int类型
// "b"表示bool类型
// "s"表示set类型
// "v"表示vector数据类型
// 'l'表示list数据类型
// "KF"表示KeyPoint数据类型
namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, shared_ptr<PointCloudMapping> pPointCloud, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpPointCloudMapping( pPointCloud ),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    //       |fx  0   cx|
    // K = |0   fy  cy|
    //       |0   0   1 |
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    // 图像矫正系数
    // [k1 k2 p1 p2 k3]
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    // 双目摄像头baseline * fx 50
    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    // 1:RGB 0:BGR
    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    // 每一帧提取的特征点数 1000
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    // 图像建立金字塔时的变化尺庄1�71ￄ1�77 1.2
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    // 尺度金字塔的层数 8
    int nLevels = fSettings["ORBextractor.nLevels"];
    // 提取fast特征点的默认阈�ￄ1�71ￄ1�77 20
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    // 如果默认阈��提取不出足够fast特征点，则使用最小阈倄1�71ￄ1�77 8
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    // tracking过程都会用到mpORBextractorLeft作为特征点提取器
    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // 如果是双目，tracking过程中还会用用到mpORBextractorRight作为右目特征点提取器
    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // 在单目初始化的时候，会用mpIniORBextractor来作为特征点提取噄1�71ￄ1�77,比上面多丢�倍特征点敄1�71ￄ1�77
    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        // 判断丢�丄1�71ￄ1�773D点远/近的阈�ￄ1�71ￄ1�77 mbf * 35 / fx
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        // 深度相机disparity转化为depth时的因子
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)//求绝对�ￄ1�71ￄ1�77
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

// 输入左右目图像，可以为RGB、BGR、RGBA、GRAY
// 1、将图像转为mImGray和imGrayRight并初始化mCurrentFrame
// 2、进行tracking过程
// 输出世界坐标系到该帧相机坐标系的变换矩阵
cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_BGRA2GRAY);
        }
    }
    // 步骤2：构造Frame,mCurrentFrame构�ￄ1�71ￄ1�77
    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    // 步骤3：跟踄1�71ￄ1�77
    Track();

    return mCurrentFrame.mTcw.clone();
}

// 输入左目RGB或RGBA图像和深度图
// 1、将图像转为mImGray和imDepth并初始化mCurrentFrame
// 2、进行tracking过程
// 输出世界坐标系到该帧相机坐标系的变换矩阵
cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImRGB = imRGB;
    mImGray = imRGB;
    mImDepth = imD;
    // 步骤1：将RGB或RGBA图像转为灰度图像
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }
    // 步骤2：将深度相机的disparity转为Depth
    if((fabs(mDepthMapFactor-1.0f)>1e-5) || mImDepth.type()!=CV_32F)
        mImDepth.convertTo(mImDepth,CV_32F,mDepthMapFactor);
    // 步骤3：构造Frame
    mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    // 步骤4：跟踄1�71ￄ1�77
    Track();

    return mCurrentFrame.mTcw.clone();
}

// 输入左目RGB或RGBA图像
// 1、将图像转为mImGray并初始化mCurrentFrame
// 2、进行tracking过程
// 输出世界坐标系到该帧相机坐标系的变换矩阵
cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;
    mImRGB = im;

    // 步骤1：将RGB或RGBA图像转为灰度图像
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }
    // 步骤2：构造Frame
    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)// 没有成功初始化的前一个状态就是NO_IMAGES_YET
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    // 步骤3：跟踄1�71ￄ1�77
    Track();

    return mCurrentFrame.mTcw.clone();
}
/**
 * @brief Main tracking function. It is independent of the input sensor.
 *
 * Tracking 线程
 */
void Tracking::Track()
{
    // track包含两部分：估计运动、跟踪局部地囄1�71ￄ1�77
    
    // mState为tracking的状态机
    // SYSTME_NOT_READY, NO_IMAGE_YET, NOT_INITIALIZED, OK, LOST
    // 如果图像复位过��或者第丢�次运行，则为NO_IMAGE_YET状�ￄ1�71ￄ1�77
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;//判断系统是否初始化，没有初始化就跳转到初始化函数，这是单目所独有的过程；
    }
    // mLastProcessedState存储了Tracking朢�新的状��，用于FrameDrawer中的绘制
    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
    // 步骤1：初始化
    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else// 步骤2：跟踄1�71ￄ1�77
    {
        // System is initialized. Track Frame.
        // bOK为临时变量，用于表示每个函数是否执行成功
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        // 在viewer中有个开关menuLocalizationMode，有它控制是否ActivateLocalizationMode，并朢�终管控mbOnlyTracking
        // mbOnlyTracking等于false表示正常VO模式（有地图更新），mbOnlyTracking等于true表示用户手动选择定位模式
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            // 正常初始化成劄1�71ￄ1�77
            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                // 棢�查并更新上一帧被替换的MapPoints
                // 更新Fuse函数和SearchAndFuse函数替换的MapPoints
                CheckReplacedInLastFrame();//更新朢�近一帄1�71ￄ1�77 lastframe 路标炄1�71ￄ1�77
                // 步骤2.1：跟踪上丢�帧或者参考帧或��重定位

                // 运动模型是空的或刚完成重定位
                // mCurrentFrame.mnId<mnLastRelocFrameId+2这个判断不应该有
                // 应该只要mVelocity不为空，就优先��择TrackWithMotionModel
                // mnLastRelocFrameId上一次重定位的那丢�帄1�71ￄ1�77
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    // 将上丢�帧的位姿作为当前帧的初始位姿
                    // 通过BoW的方式在参��帧中找当前帧特征点的匹配点
                    // 优化每个特征点都对应3D点重投影误差即可得到位姿
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    // 根据恒��模型设定当前帧的初始位姄1�71ￄ1�77
                    // 通过投影的方式在参��帧中找当前帧特征点的匹配点
                    // 优化每个特征点所对应3D点的投影误差即可得到位姿
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        // 当使用运动模式匹配到的特征点数较少时，就会��用关键帧模式��即尝试和最近一个关键帧去做匹配。为了快速匹配，本文利用了bag of words（BoW）来加����首先，计算当前帧的BoW，并设定初始位姿为上丢�帧的位姿；其次，根据位姿和BoW词典来寻找特征匹配，使用函数matcher.SearchByBoW()；最后，利用匹配的特征优化位姿��TrackReferenceKeyFrame是跟踪参考帧，不能根据固定运动��度模型预测当前帧的位姿态，通过bow加��匹配（SearchByBow＄1�71ￄ1�77
                        // 朢�后��过优化得到优化后的位姿
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                // BOW搜索，PnP求解位姿
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated
            // 只进行跟踪tracking，局部地图不工作
 
            // 步骤2.1：跟踪上丢�帧或者参考帧或��重定位

            // tracking跟丢亄1�71ￄ1�77
            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                // mbVO是mbOnlyTracking为true时的才有的一个变釄1�71ￄ1�77
                // mbVO为false表示此帧匹配了很多的MapPoints，跟踪很正常＄1�71ￄ1�77
                // mbVO为true表明此帧匹配了很少的MapPoints，少亄1�71ￄ1�7710个，要跪的节奄1�71ￄ1�77
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    // mbVO丄1�71ￄ1�770则表明此帧匹配了很多的1�71ￄ1�773D map点，非常奄1�71ￄ1�77
                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                        // 这个地方是不是应该加上：
                        // if(!bOK)
                        //    bOK = TrackReferenceKeyFrame();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.
                    // mbVO丄1�71ￄ1�771，则表明此帧匹配了很少的3D map点，少于10个，要跪的节奏，既做跟踪又做定位

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        // 这三行没啥用＄1�71ￄ1�77
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    // 重定位没有成功，但是如果跟踪成功
                    if(bOKMM && !bOKReloc)
                    {
                        // 这三行没啥用＄1�71ￄ1�77
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            // 这段代码是不是有点多余？应该放到TrackLocalMap函数中统丢�偄1�71ￄ1�77
                            // 更新当前帧的MapPoints被观测程庄1�71ￄ1�77
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)// 只要重定位成功整个跟踪过程正常进行（定位与跟踪，更相信重定位＄1�71ￄ1�77
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }
        // 将最新的关键帧作为reference frame
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        // 步骤2.2：在帧间匹配得到初始的姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姄1�71ￄ1�77
        // local map:当前帧��当前帧的MapPoints、当前关键帧与其它关键帧共视关系
        // 在步骄1�71ￄ1�772.1中主要是两两跟踪（恒速模型跟踪上丢�帧��跟踪参考帧），这里搜索屢�部关键帧后搜集所有局部MapPoints＄1�71ￄ1�77
        // 然后将局部MapPoints和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            // 重定位成劄1�71ￄ1�77
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                // 步骤2.3：更新恒速运动模型TrackWithMotionModel中的mVelocity
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;// Tcl
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            // 步骤2.4：清除UpdateLastFrame中为当前帧临时添加的MapPoints
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    // 排除UpdateLastFrame函数中为了跟踪增加的MapPoints
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            // 步骤2.5：清除临时的MapPoints，这些MapPoints在TrackWithMotionModel的UpdateLastFrame函数里生成（仅双目和rgbd＄1�71ￄ1�77
            // 步骤2.4中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
            // 这里生成的仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图丄1�71ￄ1�77
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            // 这里不仅仅是清除mlpTemporalPoints，��过delete pMP还删除了指针指向的MapPoint
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            // 步骤2.6：检测并插入关键帧，对于双目会产生新的MapPoints
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();
            idk++;//上个版本木有
            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            // 删除那些在bundle adjustment中检测为outlier的1�71ￄ1�773D map炄1�71ￄ1�77
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        // 跟踪失败，并且relocation也没有搞定，只能重新Reset
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // 保存上一帧的数据
        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    // 步骤3：记录位姿信息，用于轨迹复现
    if(!mCurrentFrame.mTcw.empty())
    {
        // 计算相对姿��T_currentFrame_referenceKeyFrame
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        // 如果跟踪失败，则相对位姿使用上一次�ￄ1�71ￄ1�77
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}
/**
 * @brief 双目和rgbd的地图初始化
 *
 * 由于具有深度信息，直接生成MapPoints
 */

void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        // 步骤1：设定初始位姄1�71ￄ1�77
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));//对角矩阵

        // Create KeyFrame
        // 步骤2：将当前帧构造为初始关键帄1�71ￄ1�77
        // mCurrentFrame的数据类型为Frame
        // KeyFrame包含Frame、地囄1�71ￄ1�773D点��以及BoW
        // KeyFrame里有丢�个mpMap，Tracking里有丢�个mpMap，��KeyFrame里的mpMap都指向Tracking里的这个mpMap
        // KeyFrame里有丢�个mpKeyFrameDB，Tracking里有丢�个mpKeyFrameDB，��KeyFrame里的mpMap都指向Tracking里的这个mpKeyFrameDB
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        // KeyFrame中包含了地图、反过来地图中也包含了KeyFrame，相互包各1�71ￄ1�77
        // 步骤3：在地图中添加该初始关键帄1�71ￄ1�77
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        // 步骤4：为每个特征点构造MapPoint
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                // 步骤4.1：��过反投影得到该特征点的3D坐标
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                // 步骤4.2：将3D点构造为MapPoint
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);

                // 步骤4.3：为该MapPoint添加属��：
                // a.观测到该MapPoint的关键帧
                // b.该MapPoint的描述子
                // c.该MapPoint的平均观测方向和深度范围

                // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测刄1�71ￄ1�77
                pNewMP->AddObservation(pKFini,i);//输入当前KeyFrame以及第i个特征点
                // 步骤4.5：表示该KeyFrame的哪个特征点可以观测到哪丄1�71ￄ1�773D炄1�71ￄ1�77
                pKFini->AddMapPoint(pNewMP,i);
                // b.从众多观测到该MapPoint的特征点中挑选区分读朢�高的描述子，上一步每个关键帧都拥有一个特征点
                pNewMP->ComputeDistinctiveDescriptors();
                // c.更新该MapPoint平均观测方向以及观测距离的范囄1�71ￄ1�77
        /*地图点到扢�有观测到的关键帧相机中心向量，归丢�化后相加〄1�71ￄ1�77
        深度范围：地图点到参考帧（只有一帧）相机中心距离，乘上参考帧中描述子获取时金字塔放大尺度，得到最大距离mfMaxDistance＄1�71ￄ1�77
        朢�大距离除以整个金字塔朢�高层的放大尺度得到最小距离mfMinDistance。��常来说，距离较近的地图点，
        将在金字塔层数较高的地方提取出，距离较远的地图点，在金字塔层数较低的地方提取出（金字塔层数越低，分辨率越高，才能识别出远点）
        。因此��过地图点的信息（主要是对应描述子），我们可以获得该地图点对应的金字塔层级：
        const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
        从��预测该地图点在仢�么距离范围内能够被观测到＄1�71ￄ1�77*/
                pNewMP->UpdateNormalAndDepth();
                // 步骤4.4：在地图中添加该MapPoint
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;
        // 步骤4：在屢�部地图中添加该初始关键帧
        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);//作为朢�近一帄1�71ￄ1�77
        mnLastKeyFrameId=mCurrentFrame.mnId;//朢�近一个关键帧的id
        mpLastKeyFrame = pKFini;//将上述关键帧操作作为朢�近一个关键帧

        mvpLocalKeyFrames.push_back(pKFini);//屢�部关键帧数组
        mvpLocalMapPoints=mpMap->GetAllMapPoints();//屢�部关键帧关键点路标集各1�71ￄ1�77
        mpReferenceKF = pKFini;//关键帧作为参考帧
        mCurrentFrame.mpReferenceKF = pKFini;//关键帧作为参考帧？？？？？？？？？？＄1�71ￄ1�77

        // 把当前（朢�新的）局部MapPoints作为ReferenceMapPoints
        // ReferenceMapPoints是DrawMapPoints函数画图的时候用的1�71ￄ1�77
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}
/**
 * @brief 单目的地图初始化
 *
 * 并行地计算基硢�矩阵和单应��矩阵，选取其中丢�个模型，恢复出最弢�始两帧之间的相对姿��以及点亄1�71ￄ1�77
 * 得到初始两帧的匹配��相对运动��初始MapPoints
 */
void Tracking::MonocularInitialization()
{
    // 如果单目初始器还没有被创建，则创建单目初始器
    if(!mpInitializer)
    {
        // Set Reference Frame
        // 单目初始帧的特征点数必须大于100
        if(mCurrentFrame.mvKeys.size()>100)
        {
            // 步骤1：得到用于初始化的第丢�帧，初始化需要两帄1�71ￄ1�77
            mInitialFrame = Frame(mCurrentFrame);
            // 记录朢�近的丢�帄1�71ￄ1�77
            mLastFrame = Frame(mCurrentFrame);
            // mvbPrevMatched朢�大的情况就是扢�有特征点都被跟踪丄1�71ￄ1�77
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            // 这两句是多余的1�71ￄ1�77
            if(mpInitializer)
                delete mpInitializer;

            // 由当前帧构��初始器 sigma:1.0 iterations:200
            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        // 步骤2：如果当前帧特征点数大于100，则得到用于单目初始化的第二帄1�71ￄ1�77
        // 如果当前帧特征点太少，重新构造初始器
        // 因此只有连续两帧的特征点个数都大亄1�71ￄ1�77100时，才能继续进行初始化过稄1�71ￄ1�77
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        // 步骤3：在mInitialFrame与mCurrentFrame中找匹配的特征点寄1�71ￄ1�77
        // mvbPrevMatched为前丢�帧的特征点，存储了mInitialFrame中哪些点将进行接下来的匹酄1�71ￄ1�77
        // mvIniMatches存储mInitialFrame,mCurrentFrame之间匹配的特征点
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        // 步骤4：如果初始化的两帧之间的匹配点太少，重新初始匄1�71ￄ1�77
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
        // 步骤5：��过H模型或F模型进行单目初始化，得到两帧间相对运动��初始MapPoints
        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            // 步骤6：删除那些无法进行三角化的匹配点
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            // 将初始化的第丢�帧作为世界坐标系，因此第丢�帧变换矩阵为单位矩阵
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            // 由Rcw和tcw构��Tcw,并赋值给mTcw，mTcw为世界坐标系到该帧的变换矩阵
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            // 步骤6：将三角化得到的3D点包装成MapPoints
            // Initialize函数会得到mvIniP3D＄1�71ￄ1�77
            // mvIniP3D是cv::Point3f类型的一个容器，是个存放3D点的临时变量＄1�71ￄ1�77
            // CreateInitialMapMonocular射1�71ￄ1�773D点包装成MapPoint类型存入KeyFrame和Map丄1�71ￄ1�77
            CreateInitialMapMonocular();
        }
    }
}
/**
 * @brief CreateInitialMapMonocular
 *
 * 为单目摄像头三角化生成MapPoints
 */
void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    // 步骤1：将初始关键帧的描述子转为BoW
    pKFini->ComputeBoW();
    // 步骤2：将当前关键帧的描述子转为BoW
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    // 步骤3：将关键帧插入到地图
    // 凡是关键帧，都要插入地图
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    // 步骤4：将3D点包装成MapPoints
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        // 步骤4.1：用3D点构造MapPoint
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);
        // 步骤4.2：为该MapPoint添加属��：
        // a.观测到该MapPoint的关键帧
        // b.该MapPoint的描述子
        // c.该MapPoint的平均观测方向和深度范围

        // 步骤4.3：表示该KeyFrame的哪个特征点可以观测到哪丄1�71ￄ1�773D炄1�71ￄ1�77
        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);
        // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测刄1�71ￄ1�77
        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);
        // b.从众多观测到该MapPoint的特征点中挑选区分读朢�高的描述孄1�71ￄ1�77
        pMP->ComputeDistinctiveDescriptors();
        // c.更新该MapPoint平均观测方向以及观测距离的范囄1�71ￄ1�77
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        // 步骤4.4：在地图中添加该MapPoint
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    // 步骤5：更新关键帧间的连接关系
    // 圄1�71ￄ1�773D点和关键帧之间建立边，每个边有一个权重，边的权重是该关键帧与当前帧公兄1�71ￄ1�773D点的个数
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;
    // 步骤5：BA优化
    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    // 步骤6＄1�71ￄ1�77!!!将MapPoints的中值深度归丢�化到1，并归一化两帧之间变捄1�71ￄ1�77
    // 评估关键帧场景深度，q=2表示中�ￄ1�71ￄ1�77
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    // x/z y/z 将z归一化到1 
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    // 抄1�71ￄ1�773D点的尺度也归丢�化到1
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }
    // 这部分和SteroInitialization()相似
    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;// 初始化成功，至此，初始化过程完成
}
/**
 * @brief 棢�查上丢�帧中的MapPoints是否被替捄1�71ￄ1�77
 * 
 * Local Mapping线程可能会将关键帧中某些MapPoints进行替换，由于tracking中需要用到mLastFrame，这里检查并更新上一帧中被替换的MapPoints
 * @see LocalMapping::SearchInNeighbors()
 */
void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

/**
 * @brief 对参考关键帧的MapPoints进行跟踪
 * 
 * 1. 计算当前帧的词包，将当前帧的特征点分到特定层的nodes丄1�71ￄ1�77
 * 2. 对属于同丢�node的描述子进行匹配
 * 3. 根据匹配对估计当前帧的姿怄1�71ￄ1�77
 * 4. 根据姿��剔除误匹配
 * @return 如果匹配数大亄1�71ￄ1�7710，返回true
 */
bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    // 步骤1：将当前帧的描述子转化为BoW向量
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);//定义ORB匹配
    vector<MapPoint*> vpMapPointMatches;//定义匹配到的路标炄1�71ￄ1�77

    // 步骤2：��过特征点的BoW加快当前帧与参��帧之间的特征点匹配
    // 特征点的匹配关系由MapPoints进行维护
    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);//获取匹配的对敄1�71ￄ1�77

    if(nmatches<15)
        return false;
    // 步骤3:将上丢�帧的位姿态作为当前帧位姿的初始�ￄ1�71ￄ1�77
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);// 用上丢�次的Tcw设置初��，在PoseOptimization可以收敛快一亄1�71ￄ1�77
    // 步骤4:通过优化3D-2D的重投影误差来获得位姄1�71ￄ1�77
    Optimizer::PoseOptimization(&mCurrentFrame);//优化

    // Discard outliers
    // 步骤5：剔除优化后的outlier匹配点（MapPoints＄1�71ￄ1�77
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}
/**
 * @brief 双目或rgbd摄像头根据深度��为上一帧产生新的MapPoints
 *
 * 在双目和rgbd情况下，选取丢�些深度小丢�些的点（可靠丢�些） \n
 * 可以通过深度值产生一些新的MapPoints
 */
void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    // 步骤1：更新最近一帧的位姿
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());// Tlr*Trw = Tlw 1:last r:reference w:world

    // 如果上一帧为关键帧，或��单目的情况，则逢�凄1�71ￄ1�77
    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // 步骤2：对于双目或rgbd摄像头，为上丢�帧临时生成新的MapPoints
    // 注意这些MapPoints不加入到Map中，在tracking的最后会删除
    // 跟踪过程中需要将将上丢�帧的MapPoints投影到当前帧可以缩小匹配范围，加快当前帧与上丢�帧进行特征点匹配

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    // 步骤2.1：得到上丢�帧有深度值的特征炄1�71ￄ1�77
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    // 步骤2.2：按照深度从小到大排庄1�71ￄ1�77
    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    // 步骤2.3：将距离比较近的点包装成MapPoints
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            // 这些生成MapPoints后并没有通过＄1�71ￄ1�77
            // a.AddMapPoint〄1�71ￄ1�77
            // b.AddObservation〄1�71ￄ1�77
            // c.ComputeDistinctiveDescriptors〄1�71ￄ1�77
            // d.UpdateNormalAndDepth添加属��，
            // 这些MapPoint仅仅为了提高双目和RGBD的跟踪成功率
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);//获取该特征点三维坐标
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;// 添加新的MapPoint

            // 标记为临时添加的MapPoint，之后在CreateNewKeyFrame之前会全部删附1�71ￄ1�77
            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}
/**
 * @brief 根据匢�速度模型对上丢�帧的MapPoints进行跟踪
 * 
 * 1. 非单目情况，霢�要对上一帧产生一些新的MapPoints（临时）
 * 2. 将上丢�帧的MapPoints投影到当前帧的图像平面上，在投影的位置进行区域匹酄1�71ￄ1�77
 * 3. 根据匹配对估计当前帧的姿怄1�71ￄ1�77
 * 4. 根据姿��剔除误匹配
 * @return 如果匹配数大亄1�71ￄ1�7710，返回true
 * @see V-B Initial Pose Estimation From Previous Frame
 */
bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    // 步骤1：对于双目或rgbd摄像头，根据深度值为上一关键帧生成新的MapPoints
    // （跟踪过程中霢�要将当前帧与上一帧进行特征点匹配，将上一帧的MapPoints投影到当前帧可以缩小匹配范围＄1�71ￄ1�77
    // 在跟踪过程中，去除outlier的MapPoint，如果不及时增加MapPoint会��渐减少
    // 这个函数的功能就是补充增加RGBD和双目相机上丢�帧的MapPoints敄1�71ￄ1�77
    UpdateLastFrame();

    // 根据Const Velocity Model(认为这两帧之间的相对运动和之前两帧间相对运动相同)估计当前帧的位姿，nani
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    // 步骤2：根据匀速度模型进行对上丢�帧的MapPoints进行跟踪
    // 根据上一帧特征点对应的1�71ￄ1�773D点投影的位置缩小特征点匹配范囄1�71ￄ1�77
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    // 如果跟踪的点少，则扩大搜索半径再来一欄1�71ￄ1�77
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    // 步骤3：优化位姄1�71ￄ1�77
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    // 步骤4：优化位姿后剔除outlier的mvpMapPoints
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}
/**
 * @brief 对Local Map的MapPoints进行跟踪
 * 
 * 1. 更新屢�部地图，包括屢�部关键帧和关键点
 * 2. 对局部MapPoints进行投影匹配
 * 3. 根据匹配对估计当前帧的姿怄1�71ￄ1�77
 * 4. 根据姿��剔除误匹配
 * @return true if success
 * @see V-D track Local Map
 */
bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    // Update Local KeyFrames and Local Points
    // 步骤1：更新局部关键帧mvpLocalKeyFrames和局部地图点mvpLocalMapPoints
    UpdateLocalMap();

    // 步骤2：在屢�部地图中查找与当前帧匹配的MapPoints
    SearchLocalPoints();

    // Optimize Pose
    // 在这个函数之前，在Relocalization、TrackReferenceKeyFrame、TrackWithMotionModel中都有位姿优化，
    // 步骤3：更新局部所有MapPoints后对位姿再次优化
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    // 步骤3：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            // 由于当前帧的MapPoints可以被当前帧观测到，其被观测统计量加1
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    // 该MapPoint被其它关键帧观测到过
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    // 记录当前帧跟踪到的MapPoints，用于统计跟踪效构1�71ￄ1�77
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    // 步骤4：决定是否跟踪成劄1�71ￄ1�77
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}

/**
 * @brief 断当前帧是否为关键帧
 * @return true if needed
 */
bool Tracking::NeedNewKeyFrame()
{
    // 步骤1：如果用户在界面上��择重定位，那么将不插入关键帄1�71ￄ1�77
    // 由于插入关键帧过程中会生成MapPoint，因此用户��择重定位后地图上的点云和关键帧都不会再增加
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    // 如果屢�部地图被闭环棢�测使用，则不插入关键帄1�71ￄ1�77
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    // 步骤2：判断是否距离上丢�次插入关键帧的时间太矄1�71ￄ1�77
    // mCurrentFrame.mnId是当前帧的ID
    // mnLastRelocFrameId是最近一次重定位帧的ID
    // mMaxFrames等于图像输入的帧玄1�71ￄ1�77
    // 如果关键帧比较少，则考虑插入关键帄1�71ￄ1�77
    // 或距离上丢�次重定位超过1s，则考虑插入关键帄1�71ￄ1�77
    // if(mCurrentFrame.mnId < mnLastRelocFrameId + 2 * mMaxFrames)
    //     return false;
    if(mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    // 步骤3：得到参考关键帧跟踪到的MapPoints数量
    // 在UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参��关键帧
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    // 步骤4：查询局部地图管理器是否繁忙
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();
    /*
    // Stereo & RGB-D: Ratio of close "matches to map"/"total matches"
    // "total matches = matches to map + visual odometry matches"
    // Visual odometry matches will become MapPoints if we insert a keyframe.
    // This ratio measures how many MapPoints we could create if we insert a keyframe.
    // 步骤5：对于双目或RGBD摄像头，统计总的可以添加的MapPoints数量和跟踪到地图中的MapPoints数量
    int nMap = 0;
    int nTotal= 0;
    if(mSensor!=System::MONOCULAR)// 双目或rgbd
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                nTotal++;// 总的可以添加mappoints敄1�71ￄ1�77
                if(mCurrentFrame.mvpMapPoints[i])
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        nMap++;// 被关键帧观测到的mappoints数，即观测到地图中的MapPoints数量
            }
        }
    }
    else
    {
        // There are no visual odometry matches in the monocular case
        nMap=1;
        nTotal=1;
    }

    const float ratioMap = (float)nMap/(float)(std::max(1,nTotal));*/
    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // 步骤6：决策是否需要插入关键帧
    // Thresholds
    // 设定inlier阈��，和之前帧特征点匹配的inlier比例
    float thRefRatio = 0.7f;
    if(nKFs<2)
        thRefRatio = 0.4f;// 关键帧只有一帧，那么插入关键帧的阈��设置很佄1�71ￄ1�77

    if(mSensor==System::MONOCULAR)
        // thRefRatio = 0.9f;
        thRefRatio = 0.7f;

    /*
    // MapPoints中和地图关联的比例阈倄1�71ￄ1�77
    float thMapRatio = 0.35f;
    if(mnMatchesInliers>300)
        thMapRatio = 0.20f;*/

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    // 很长时间没有插入关键帄1�71ￄ1�77
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId + 1.5 * mMaxFrames;  
    if(mSensor==System::MONOCULAR)
        const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+1.5*mMaxFrames;

    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    // localMapper处于空闲状�ￄ1�71ￄ1�77
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    // 跟踪要跪的节奏，0.25咄1�71ￄ1�770.3是一个比较低的阈倄1�71ￄ1�77
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    // 阈��比c1c要高，与之前参��帧（最近的丢�个关键帧）重复度不是太高
    // const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);


    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                // 队列里不能阻塞太多关键帧
                // tracking插入关键帧不是直接插入，而且先插入到mlNewKeyFrames中，
                // 然后localmapper再��个pop出来插入到mspKeyFrames
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}
/**
 * @brief 创建新的关键帄1�71ￄ1�77
 *
 * 对于非单目的情况，同时创建新的MapPoints
 */
void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;
    // 步骤1：将当前帧构造成关键帄1�71ￄ1�77
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    // 步骤2：将当前关键帧设置为当前帧的参��关键帧
    // 在UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参��关键帧
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    // 这段代码和UpdateLastFrame中的那一部分代码功能相同
    // 步骤3：对于双目或rgbd摄像头，为当前帧生成新的MapPoints
    if(mSensor!=System::MONOCULAR)
    {
        // 根据Tcw计算mRcw、mtcw和mRwc、mOw
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        // 步骤3.1：得到当前帧深度小于阈��的特征炄1�71ￄ1�77
        // 创建新的MapPoint, depth < mThDepth
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            // 步骤3.2：按照深度从小到大排庄1�71ￄ1�77
            sort(vDepthIdx.begin(),vDepthIdx.end());

            // 步骤3.3：将距离比较近的点包装成MapPoints
            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    // 这些添加属��的操作是每次创建MapPoint后都要做的1�71ￄ1�77
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                // 这里决定了双目和rgbd摄像头时地图点云的稠密程庄1�71ￄ1�77
                // 但是仅仅为了让地图稠密直接改这些不太好，
                // 因为这些MapPoints会参与之后整个slam过程
                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    std::cout << "KeyFrame size = " << vpKFs.size() << std::endl;
    // insert Key Frame into point cloud viewer
    if(mSensor==System::RGBD)
        mpPointCloudMapping->insertKeyFrame( pKF, this->mImRGB, this->mImDepth ,idk, vpKFs);
    if(mSensor==System::MONOCULAR)
    {
        mpPointCloudMapping->insertKeyFramemono( pKF, this->mImRGB, idk, vpKFs);
    }

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}
/**
 * @brief 对Local MapPoints进行跟踪
 * 
 * 在局部地图中查找在当前帧视野范围内的点，将视野范围内的点和当前帧的特征点进行投影匹配
 */
void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    // 步骤1：遍历当前帧的mvpMapPoints，标记这些MapPoints不参与之后的搜索
    // 因为当前的mvpMapPoints丢�定在当前帧的视野丄1�71ￄ1�77
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                // 更新能观测到该点的帧数加1
                pMP->IncreaseVisible();
                // 标记该点被当前帧观测刄1�71ￄ1�77
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                // 标记该点将来不被投影，因为已经匹配过
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    // 步骤2：将扢�有局部MapPoints投影到当前帧，判断是否在视野范围内，然后进行投影匹配
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        // 已经被当前帧观测到MapPoint不再判断是否能被当前帧观测到
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        // 步骤2.1：判断LocalMapPoints中的点是否在在视野内
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            // 观测到该点的帧数劄1�71ￄ1�771，该MapPoint在某些帧的视野范围内
            pMP->IncreaseVisible();
            // 只有在视野范围内的MapPoints才参与之后的投影匹配
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        // 如果不久前进行过重定位，那么进行丢�个更加宽泛的搜索，阈值需要增处1�71ￄ1�77
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        // 步骤2.2：对视野范围内的MapPoints通过投影进行特征点匹酄1�71ￄ1�77
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}
/**
 * @brief 更新LocalMap
 *
 * 屢�部地图包括： \n
 * - K1个关键帧、K2个临近关键帧和参考关键帧
 * - 由这些关键帧观测到的MapPoints
 */
void Tracking::UpdateLocalMap()
{
    // This is for visualization
    // 这行程序放在UpdateLocalPoints函数后面是不是好丢�亄1�71ￄ1�77
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    // 更新屢�部关键帧和局部MapPoints
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

/**
 * @brief 更新屢�部关键点，called by UpdateLocalMap()
 * 
 * 屢�部关键帧mvpLocalKeyFrames的MapPoints，更新mvpLocalMapPoints
 */
void Tracking::UpdateLocalPoints()
{
    // 步骤1：清空局部MapPoints
    mvpLocalMapPoints.clear();

    // 步骤2：遍历局部关键帧mvpLocalKeyFrames
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        // 步骤2：将屢�部关键帧的MapPoints添加到mvpLocalMapPoints
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            // mnTrackReferenceForFrame防止重复添加屢�部MapPoint
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


/**
 * @brief 更新屢�部关键帧，called by UpdateLocalMap()
 *
 * 遍历当前帧的MapPoints，将观测到这些MapPoints的关键帧和相邻的关键帧取出，更新mvpLocalKeyFrames
 */
void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    // 步骤1：遍历当前帧的MapPoints，记录所有能观测到当前帧MapPoints的关键帧
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                // 能观测到当前帧MapPoints的关键帧
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    // 步骤2：更新局部关键帧（mvpLocalKeyFrames），添加屢�部关键帧有三个策畄1�71ￄ1�77
    // 先清空局部关键帧
    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    // V-D K1: shares the map points with current frame
    // 策略1：能观测到当前帧MapPoints的关键帧作为屢�部关键帧
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        // mnTrackReferenceForFrame防止重复添加屢�部关键帧
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    // V-D K2: neighbors to K1 in the covisibility graph
    // 策略2：与策略1得到的局部关键帧共视程度很高的关键帧作为屢�部关键帧
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        // 策略2.1:朢�佳共视的10帄1�71ￄ1�77
        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                // mnTrackReferenceForFrame防止重复添加屢�部关键帧
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // 策略2.2:自己的子关键帄1�71ￄ1�77
        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // 策略2.3:自己的父关键帄1�71ￄ1�77
        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            // mnTrackReferenceForFrame防止重复添加屢�部关键帧
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    // V-D Kref＄1�71ￄ1�77 shares the most map points with current frame
    // 步骤3：更新当前帧的参考关键帧，与自己共视程度朢�高的关键帧作为参考关键帧
    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    // 步骤1：计算当前帧特征点的Bow映射，下面函数要甄1�71ￄ1�77
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    // 步骤2：找到与当前帧相似的候��关键帧
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;//关键帧（向量）的路标点（向量＄1�71ￄ1�77
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    //bow对每丢�个����关键帧匹配
    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            // 步骤3：��过BoW进行匹配
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                // 初始化PnPsolver
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            // 步骤4：��过EPnP算法估计姿�ￄ1�71ￄ1�77
            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;//从小到大排列的容噄1�71ￄ1�77 

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                // 步骤5：��过PoseOptimization对姿态进行优化求规1�71ￄ1�77
                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                // 步骤6：如果内点较少，则��过投影的方式对之前未匹配的点进行匹配，再进行优化求规1�71ￄ1�77
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
