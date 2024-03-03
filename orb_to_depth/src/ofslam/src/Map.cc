

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}
/**
 * @brief Insert KeyFrame in the map
 * @param pKF KeyFrame
 */
void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}
/**
 * @brief Insert MapPoint in the map
 * @param pMP MapPoint
 */
void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}
/**
 * @brief Erase MapPoint from the map
 * @param pMP MapPoint
 */
void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}
/**
 * @brief Erase KeyFrame from the map
 * @param pKF KeyFrame
 */
void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}
/**
 * @brief 璁剧疆鍙傝€僊apPoints锛屽皢鐢ㄤ簬DrawMapPoints鍑芥暟鐢诲浘
 * @param vpMPs Local MapPoints
 */
void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());//std::set<KeyFrame*> mspKeyFrames;
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());//
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

// 对关键帧相关数据进行保存
void Map::Save(const string &filename,const cv::MatSize image_size)
{
    std::cout << "SFM Saving to "<< filename << std::endl;
    ofstream f;
    f.open(filename.c_str());

    f << "MVS "<< image_size[1] << " "<< image_size[0] << endl;
    // 输出关键帧的数量
    cout << "The number of KeyFrames: " << mspKeyFrames.size() << endl;

    unsigned long int nKeyFrames = mspKeyFrames.size();
    f << nKeyFrames << endl;
    for(auto kf:mspKeyFrames)
        SaveKeyFrame(f,kf);

    // 输出空间三维点的数目
    cout << "The number of MapPoints: " << mspMapPoints.size();
    unsigned long int nMapPoints = mspMapPoints.size();
    f << nMapPoints << endl;

    for(auto mp:mspMapPoints)
        SaveMapPoint(f,mp);

    f.close();

}

//  保存地图点
void Map::SaveMapPoint(ofstream &f, MapPoint *mp)
{
    //保存当前MapPoint世界坐标值
    cv::Mat mpWorldPos = mp->GetWorldPos();
    f <<" " <<mpWorldPos.at<float>(0)<<" " << mpWorldPos.at<float>(1)<<" " << mpWorldPos.at<float>(2) << " ";
    f << (mp->nObs)/2<< " ";

    std::map<KeyFrame*,size_t> mapObservation = mp->GetObservations();
    for(auto mit = mapObservation.begin(); mit != mapObservation.end(); mit++)
    {
        int Frameid;
        Frameid = mit->first->mnId;
        auto keyid = find(KeyId.begin(),KeyId.end(),Frameid) - KeyId.begin();
        f << keyid << " ";
    }
    f << "\n";
}

//  保存关键帧
void Map::SaveKeyFrame(ofstream &f, KeyFrame *kf)
{
    KeyId.push_back(kf->mnId);
    // 保存当前关键帧的id
    f << KeyId.end() - KeyId.begin() - 1<< " ";
    // 关键帧内参
    f << kf->fx << " " << kf->fy << " " << kf->cx << " " << kf->cy << " ";
    // 保存当前关键帧的位姿
    cv::Mat Tcw = kf->GetPose();
    cout << "GetPose " << std::to_string(kf->mTimeStamp) <<"\nTcw\n" <<Tcw<< endl;
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cout << "Rcw\n" << Rcw << endl;
    // 通过四元数保存旋转矩阵
    std::vector<float> Quat = Converter::toQuaternion(Rcw);

    for(int i=0; i<4; i++)
    {
        f << Quat[(3+i)%4] << " ";// qw qx qy qz
    }
    //保存平移
    for(int i=0; i<3; i++)
    {
        f << Tcw.at<float>(i,3) << " ";
    }
    ostringstream sTimeStamp;
    sTimeStamp << std::to_string(kf->mTimeStamp);
    f << sTimeStamp.str();
    f << "\n";

    // std::string inputImagePath = "/home/zjd/shujuji/city100/rgb/" + std::to_string(kf->mTimeStamp) + ".png";
    // cv::Mat image = cv::imread(inputImagePath, cv::IMREAD_UNCHANGED);
    // if (image.empty()) {
    //     std::cerr << "Failed to read image: " << inputImagePath << std::endl;
    // }

    // std::string outputImagePath = "/home/zjd/shujuji/city100orb+mvg+mvs/images/" + std::to_string(kf->mTimeStamp) + ".png";

    // if (!cv::imwrite(outputImagePath, image)) {
    //     std::cerr << "Failed to save image: " << outputImagePath << std::endl;
    // }
}
} //namespace ORB_SLAM
