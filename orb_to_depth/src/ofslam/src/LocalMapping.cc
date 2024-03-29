/**
* Used to establish a local map
*/

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include<mutex>

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)//system涓皟鐢�
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        // 鍛婅瘔Tracking锛孡ocalMapping姝ｅ浜庣箒蹇欑姸鎬侊紝
        // LocalMapping绾跨▼澶勭悊鐨勫叧閿抚閮芥槸Tracking绾跨▼鍙戣繃鐨�
        // 鍦↙ocalMapping绾跨▼杩樻病鏈夊鐞嗗畬鍏抽敭甯т箣鍓峊racking绾跨▼鏈€濂戒笉瑕佸彂閫佸お蹇�
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        // 绛夊緟澶勭悊鐨勫叧閿抚鍒楄〃涓嶄负绌�
        if(CheckNewKeyFrames())
        {
            // BoW conversion and insertion in Map
            // VI-A keyframe insertion
            // 璁＄畻鍏抽敭甯х壒寰佺偣鐨凚oW鏄犲皠锛屽皢鍏抽敭甯ф彃鍏ュ湴鍥�
            ProcessNewKeyFrame();

            // Check recent MapPoints
            // VI-B recent map points culling
            // 鍓旈櫎ProcessNewKeyFrame鍑芥暟涓紩鍏ョ殑涓嶅悎鏍糓apPoints
            MapPointCulling();

            // Triangulate new MapPoints
            // VI-C new map points creation
            // 鐩告満杩愬姩杩囩▼涓笌鐩搁偦鍏抽敭甯ч€氳繃涓夎鍖栨仮澶嶅嚭涓€浜汳apPoints
            CreateNewMapPoints();

            // 宸茬粡澶勭悊瀹岄槦鍒椾腑鐨勬渶鍚庣殑涓€涓叧閿抚
            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                // 妫€鏌ュ苟铻嶅悎褰撳墠鍏抽敭甯т笌鐩搁偦甯э紙涓ょ骇鐩搁偦锛夐噸澶嶇殑MapPoints
                SearchInNeighbors();
            }

            // 宸茬粡澶勭悊瀹岄槦鍒椾腑鐨勬渶鍚庣殑涓€涓叧閿抚锛屽苟涓旈棴鐜娴嬫病鏈夎姹傚仠姝ocalMapping
            mbAbortBA = false;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                //  VI-D Local BA
                if(mpMap->KeyFramesInMap()>2)
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);

                // Check redundant local Keyframes
                // VI-E local keyframes culling
                // 妫€娴嬪苟鍓旈櫎褰撳墠甯х浉閭荤殑鍏抽敭甯т腑鍐椾綑鐨勫叧閿抚
                // 鍓旈櫎鐨勬爣鍑嗘槸锛氳鍏抽敭甯х殑90%鐨凪apPoints鍙互琚叾瀹冨叧閿抚瑙傛祴鍒�
                // trick! 
                // Tracking涓厛鎶婂叧閿抚浜ょ粰LocalMapping绾跨▼
                // 骞朵笖鍦═racking涓璉nsertKeyFrame鍑芥暟鐨勬潯浠舵瘮杈冩澗锛屼氦缁橪ocalMapping绾跨▼鐨勫叧閿抚浼氭瘮杈冨瘑
                // 鍦ㄨ繖閲屽啀鍒犻櫎鍐椾綑鐨勫叧閿抚
                KeyFrameCulling();
            }

            // 灏嗗綋鍓嶅抚鍔犲叆鍒伴棴鐜娴嬮槦鍒椾腑
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);//std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}
/**
 * @brief 鎻掑叆鍏抽敭甯�
 *
 * 灏嗗叧閿抚鎻掑叆鍒板湴鍥句腑锛屼互渚垮皢鏉ヨ繘琛屽眬閮ㄥ湴鍥句紭鍖�
 * 杩欓噷浠呬粎鏄皢鍏抽敭甯ф彃鍏ュ埌鍒楄〃涓繘琛岀瓑寰�
 * @param pKF KeyFrame
 */
void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


/**
 * @brief 鏌ョ湅鍒楄〃涓槸鍚︽湁绛夊緟琚彃鍏ョ殑鍏抽敭甯�
 * @return 濡傛灉瀛樺湪锛岃繑鍥瀟rue
 */
bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}
/**
 * @brief 澶勭悊鍒楄〃涓殑鍏抽敭甯�
 * 
 * - 璁＄畻Bow锛屽姞閫熶笁瑙掑寲鏂扮殑MapPoints
 * - 鍏宠仈褰撳墠鍏抽敭甯ц嚦MapPoints锛屽苟鏇存柊MapPoints鐨勫钩鍧囪娴嬫柟鍚戝拰瑙傛祴璺濈鑼冨洿
 * - 鎻掑叆鍏抽敭甯э紝鏇存柊Covisibility鍥惧拰Essential鍥�
 * @see VI-A keyframe insertion
 */
void LocalMapping::ProcessNewKeyFrame()
{
    // 姝ラ1锛氫粠缂撳啿闃熷垪涓彇鍑轰竴甯у叧閿抚
    // Tracking绾跨▼鍚慙ocalMapping涓彃鍏ュ叧閿抚瀛樺湪璇ラ槦鍒椾腑
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        // 浠庡垪琛ㄤ腑鑾峰緱涓€涓瓑寰呰鎻掑叆鐨勫叧閿抚
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    // 姝ラ2锛氳绠楄鍏抽敭甯х壒寰佺偣鐨凚ow鏄犲皠鍏崇郴
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    // 姝ラ3锛氳窡韪眬閮ㄥ湴鍥捐繃绋嬩腑鏂板尮閰嶄笂鐨凪apPoints鍜屽綋鍓嶅叧閿抚缁戝畾
    // 鍦═rackLocalMap鍑芥暟涓皢灞€閮ㄥ湴鍥句腑鐨凪apPoints涓庡綋鍓嶅抚杩涜浜嗗尮閰嶏紝
    // 浣嗘病鏈夊杩欎簺鍖归厤涓婄殑MapPoints涓庡綋鍓嶅抚杩涜鍏宠仈
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                // 闈炲綋鍓嶅抚鐢熸垚鐨凪apPoints
                // 涓哄綋鍓嶅抚鍦╰racking杩囩▼璺熻釜鍒扮殑MapPoints鏇存柊灞炴€�
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    // 娣诲姞瑙傛祴
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    // 鑾峰緱璇ョ偣鐨勫钩鍧囪娴嬫柟鍚戝拰瑙傛祴璺濈鑼冨洿
                    pMP->UpdateNormalAndDepth();
                    // 鍔犲叆鍏抽敭甯у悗锛屾洿鏂�3d鐐圭殑鏈€浣虫弿杩板瓙
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    // 褰撳墠甯х敓鎴愮殑MapPoints
                    // 灏嗗弻鐩垨RGBD璺熻釜杩囩▼涓柊鎻掑叆鐨凪apPoints鏀惧叆mlpRecentAddedMapPoints锛岀瓑寰呮鏌�
                    // CreateNewMapPoints鍑芥暟涓€氳繃涓夎鍖栦篃浼氱敓鎴怣apPoints
                    // 杩欎簺MapPoints閮戒細缁忚繃MapPointCulling鍑芥暟鐨勬楠�
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }    

    // Update links in the Covisibility Graph
    // 姝ラ4锛氭洿鏂板叧閿抚闂寸殑杩炴帴鍏崇郴锛孋ovisibility鍥惧拰Essential鍥�(tree)
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    // 姝ラ5锛氬皢璇ュ叧閿抚鎻掑叆鍒板湴鍥句腑
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}
/**
 * @brief 鍓旈櫎ProcessNewKeyFrame鍜孋reateNewMapPoints鍑芥暟涓紩鍏ョ殑璐ㄩ噺涓嶅ソ鐨凪apPoints
 * @see VI-B recent map points culling
 */
void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    // 閬嶅巻绛夊緟妫€鏌ョ殑MapPoints
    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            // 姝ラ1锛氬凡缁忔槸鍧忕偣鐨凪apPoints鐩存帴浠庢鏌ラ摼琛ㄤ腑鍒犻櫎
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            // 姝ラ2锛氬皢涓嶆弧瓒砎I-B鏉′欢鐨凪apPoint鍓旈櫎
            // VI-B 鏉′欢1锛�
            // 璺熻釜鍒拌MapPoint鐨凢rame鏁扮浉姣旈璁″彲瑙傛祴鍒拌MapPoint鐨凢rame鏁扮殑姣斾緥闇€澶т簬25%
            // IncreaseFound / IncreaseVisible < 25%锛屾敞鎰忎笉涓€瀹氭槸鍏抽敭甯с€�
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            // 姝ラ3锛氬皢涓嶆弧瓒砎I-B鏉′欢鐨凪apPoint鍓旈櫎
            // VI-B 鏉′欢2锛氫粠璇ョ偣寤虹珛寮€濮嬶紝鍒扮幇鍦ㄥ凡缁忚繃浜嗕笉灏忎簬2涓叧閿抚
            // 浣嗘槸瑙傛祴鍒拌鐐圭殑鍏抽敭甯ф暟鍗翠笉瓒呰繃cnThObs甯э紝閭ｄ箞璇ョ偣妫€楠屼笉鍚堟牸
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            // 姝ラ4锛氫粠寤虹珛璇ョ偣寮€濮嬶紝宸茬粡杩囦簡3涓叧閿抚鑰屾病鏈夎鍓旈櫎锛屽垯璁や负鏄川閲忛珮鐨勭偣
            // 鍥犳娌℃湁SetBadFlag()锛屼粎浠庨槦鍒椾腑鍒犻櫎锛屾斁寮冪户缁璇apPoint鐨勬娴�
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}
/**
 * 鐩告満杩愬姩杩囩▼涓拰鍏辫绋嬪害姣旇緝楂樼殑鍏抽敭甯ч€氳繃涓夎鍖栨仮澶嶅嚭涓€浜汳apPoints
 */
void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    if(mbMonocular)
        nn=20;
    // 姝ラ1锛氬湪褰撳墠鍏抽敭甯х殑鍏辫鍏抽敭甯т腑鎵惧埌鍏辫绋嬪害鏈€楂樼殑nn甯х浉閭诲抚vpNeighKFs
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    // 寰楀埌褰撳墠鍏抽敭甯у湪涓栫晫鍧愭爣绯讳腑鐨勫潗鏍�
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    // 姝ラ2锛氶亶鍘嗙浉閭诲叧閿抚vpNeighKFs
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        // 閭绘帴鐨勫叧閿抚鍦ㄤ笘鐣屽潗鏍囩郴涓殑鍧愭爣
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        // 鍩虹嚎鍚戦噺锛屼袱涓叧閿抚闂寸殑鐩告満浣嶇Щ
        cv::Mat vBaseline = Ow2-Ow1;
        // 鍩虹嚎闀垮害
        const float baseline = cv::norm(vBaseline);

        // 姝ラ3锛氬垽鏂浉鏈鸿繍鍔ㄧ殑鍩虹嚎鏄笉鏄冻澶熼暱
        if(!mbMonocular)
        {
            // 濡傛灉鏄珛浣撶浉鏈猴紝鍏抽敭甯ч棿璺濆お灏忔椂涓嶇敓鎴�3D鐐�
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            // 閭绘帴鍏抽敭甯х殑鍦烘櫙娣卞害涓€�
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            // baseline涓庢櫙娣辩殑姣斾緥
            const float ratioBaselineDepth = baseline/medianDepthKF2;
            // 濡傛灉鐗瑰埆杩�(姣斾緥鐗瑰埆灏�)锛岄偅涔堜笉鑰冭檻褰撳墠閭绘帴鐨勫叧閿抚锛屼笉鐢熸垚3D鐐�

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        // 姝ラ4锛氭牴鎹袱涓叧閿抚鐨勪綅濮胯绠楀畠浠箣闂寸殑鍩烘湰鐭╅樀
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        // 姝ラ5锛氶€氳繃鏋佺嚎绾︽潫闄愬埗鍖归厤鏃剁殑鎼滅储鑼冨洿锛岃繘琛岀壒寰佺偣鍖归厤
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        // 姝ラ6锛氬姣忓鍖归厤閫氳繃涓夎鍖栫敓鎴�3D鐐�,he Triangulate鍑芥暟宸笉澶�
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            // 姝ラ6.1锛氬彇鍑哄尮閰嶇壒寰佺偣

            // 褰撳墠鍖归厤瀵瑰湪褰撳墠鍏抽敭甯т腑鐨勭储寮�
            const int &idx1 = vMatchedIndices[ikp].first;
            // 褰撳墠鍖归厤瀵瑰湪閭绘帴鍏抽敭甯т腑鐨勭储寮�
            const int &idx2 = vMatchedIndices[ikp].second;

            // 褰撳墠鍖归厤鍦ㄥ綋鍓嶅叧閿抚涓殑鐗瑰緛鐐�
            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            // mvuRight涓瓨鏀剧潃鍙岀洰鐨勬繁搴﹀€硷紝濡傛灉涓嶆槸鍙岀洰锛屽叾鍊煎皢涓�-1
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            // 褰撳墠鍖归厤鍦ㄩ偦鎺ュ叧閿抚涓殑鐗瑰緛鐐�
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            // mvuRight涓瓨鏀剧潃鍙岀洰鐨勬繁搴﹀€硷紝濡傛灉涓嶆槸鍙岀洰锛屽叾鍊煎皢涓�-1
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            // 姝ラ6.2锛氬埄鐢ㄥ尮閰嶇偣鍙嶆姇褰卞緱鍒拌宸
            // 鐗瑰緛鐐瑰弽鎶曞奖
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            // 鐢辩浉鏈哄潗鏍囩郴杞埌涓栫晫鍧愭爣绯伙紝寰楀埌瑙嗗樊瑙掍綑寮﹀€�
            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            // 鍔�1鏄负浜嗚cosParallaxStereo闅忎究鍒濆鍖栦负涓€涓緢澶х殑鍊�
            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            // 姝ラ6.3锛氬浜庡弻鐩紝鍒╃敤鍙岀洰寰楀埌瑙嗗樊瑙�
            if(bStereo1)//鍙岀洰锛屼笖鏈夋繁搴�
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)//鍙岀洰锛屼笖鏈夋繁搴�
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            // 寰楀埌鍙岀洰瑙傛祴鐨勮宸
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            // 姝ラ6.4锛氫笁瑙掑寲鎭㈠3D鐐�
            cv::Mat x3D;
            // cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998)琛ㄦ槑瑙嗗樊瑙掓甯�
            // cosParallaxRays<cosParallaxStereo琛ㄦ槑瑙嗗樊瑙掑緢灏�
            // 瑙嗗樊瑙掑害灏忔椂鐢ㄤ笁瑙掓硶鎭㈠3D鐐癸紝瑙嗗樊瑙掑ぇ鏃剁敤鍙岀洰鎭㈠3D鐐癸紙鍙岀洰浠ュ強娣卞害鏈夋晥锛�
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                // 瑙両nitializer.cpp鐨凾riangulate鍑芥暟
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            // 姝ラ6.5锛氭娴嬬敓鎴愮殑3D鐐规槸鍚﹀湪鐩告満鍓嶆柟
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            // 姝ラ6.6锛氳绠�3D鐐瑰湪褰撳墠鍏抽敭甯т笅鐨勯噸鎶曞奖璇樊
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                // 鍩轰簬鍗℃柟妫€楠岃绠楀嚭鐨勯槇鍊硷紙鍋囪娴嬮噺鏈変竴涓儚绱犵殑鍋忓樊锛�
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            // 璁＄畻3D鐐瑰湪鍙︿竴涓叧閿抚涓嬬殑閲嶆姇褰辫宸�
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                // 鍩轰簬鍗℃柟妫€楠岃绠楀嚭鐨勯槇鍊硷紙鍋囪娴嬮噺鏈変竴涓竴涓儚绱犵殑鍋忓樊锛�
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            // 姝ラ6.7锛氭鏌ュ昂搴﹁繛缁€�

            // 涓栫晫鍧愭爣绯讳笅锛�3D鐐逛笌鐩告満闂寸殑鍚戦噺锛屾柟鍚戠敱鐩告満鎸囧悜3D鐐�
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            // ratioDist鏄笉鑰冭檻閲戝瓧濉斿昂搴︿笅鐨勮窛绂绘瘮渚�
            const float ratioDist = dist2/dist1;
            // 閲戝瓧濉斿昂搴﹀洜瀛愮殑姣斾緥
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
            // ratioDist*ratioFactor < ratioOctave 鎴� ratioDist/ratioOctave > ratioFactor琛ㄦ槑灏哄害鍙樺寲鏄繛缁殑
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            // 姝ラ6.8锛氫笁瑙掑寲鐢熸垚3D鐐规垚鍔燂紝鏋勯€犳垚MapPoint
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            // 姝ラ6.9锛氫负璇apPoint娣诲姞灞炴€э細
            // a.瑙傛祴鍒拌MapPoint鐨勫叧閿抚
            // b.璇apPoint鐨勬弿杩板瓙
            // c.璇apPoint鐨勫钩鍧囪娴嬫柟鍚戝拰娣卞害鑼冨洿
            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            // 姝ラ6.8锛氬皢鏂颁骇鐢熺殑鐐规斁鍏ユ娴嬮槦鍒�
            // 杩欎簺MapPoints閮戒細缁忚繃MapPointCulling鍑芥暟鐨勬楠�
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}
/**
 * 妫€鏌ュ苟铻嶅悎褰撳墠鍏抽敭甯т笌鐩搁偦甯э紙涓ょ骇鐩搁偦锛夐噸澶嶇殑MapPoints
 */
void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    // 姝ラ1锛氳幏寰楀綋鍓嶅叧閿抚鍦╟ovisibility鍥句腑鏉冮噸鎺掑悕鍓峮n鐨勯偦鎺ュ叧閿抚
    // 鎵惧埌褰撳墠甯т竴绾х浉閭讳笌浜岀骇鐩搁偦鍏抽敭甯�
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);// 鍔犲叆涓€绾х浉閭诲抚
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;// 骞舵爣璁板凡缁忓姞鍏�

        // Extend to some second neighbors
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);// 瀛樺叆浜岀骇鐩搁偦甯�
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    // 姝ラ2锛氬皢褰撳墠甯х殑MapPoints鍒嗗埆涓庝竴绾т簩绾х浉閭诲抚(鐨凪apPoints)杩涜铻嶅悎
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        // 鎶曞奖褰撳墠甯х殑MapPoints鍒扮浉閭诲叧閿抚pKFi涓紝骞跺垽鏂槸鍚︽湁閲嶅鐨凪apPoints
        // 1.濡傛灉MapPoint鑳藉尮閰嶅叧閿抚鐨勭壒寰佺偣锛屽苟涓旇鐐规湁瀵瑰簲鐨凪apPoint锛岄偅涔堝皢涓や釜MapPoint鍚堝苟锛堥€夋嫨瑙傛祴鏁板鐨勶級
        // 2.濡傛灉MapPoint鑳藉尮閰嶅叧閿抚鐨勭壒寰佺偣锛屽苟涓旇鐐规病鏈夊搴旂殑MapPoint锛岄偅涔堜负璇ョ偣娣诲姞MapPoint
        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    // 鐢ㄤ簬瀛樺偍涓€绾ч偦鎺ュ拰浜岀骇閭绘帴鍏抽敭甯ф墍鏈塎apPoints鐨勯泦鍚�
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    // 姝ラ3锛氬皢涓€绾т簩绾х浉閭诲抚鐨凪apPoints鍒嗗埆涓庡綋鍓嶅抚锛堢殑MapPoints锛夎繘琛岃瀺鍚�
    // 閬嶅巻姣忎竴涓竴绾ч偦鎺ュ拰浜岀骇閭绘帴鍏抽敭甯�
    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        // 閬嶅巻褰撳墠涓€绾ч偦鎺ュ拰浜岀骇閭绘帴鍏抽敭甯т腑鎵€鏈夌殑MapPoints
        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            // 鍒ゆ柇MapPoints鏄惁涓哄潖鐐癸紝鎴栬€呮槸鍚﹀凡缁忓姞杩涢泦鍚坴pFuseCandidates
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            // 鍔犲叆闆嗗悎锛屽苟鏍囪宸茬粡鍔犲叆
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    // 姝ラ4锛氭洿鏂板綋鍓嶅抚MapPoints鐨勬弿杩板瓙锛屾繁搴︼紝瑙傛祴涓绘柟鍚戠瓑灞炴€�
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                // 鍦ㄦ墍鏈夋壘鍒皃MP鐨勫叧閿抚涓紝鑾峰緱鏈€浣崇殑鎻忚堪瀛�
                pMP->ComputeDistinctiveDescriptors();
                // 鏇存柊骞冲潎瑙傛祴鏂瑰悜鍜岃娴嬭窛绂�
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    // 姝ラ5锛氭洿鏂板綋鍓嶅抚鐨凪apPoints鍚庢洿鏂颁笌鍏跺畠甯х殑杩炴帴鍏崇郴
    // 鏇存柊covisibility鍥�
    mpCurrentKeyFrame->UpdateConnections();
}
/**
 * 鏍规嵁涓ゅ叧閿抚鐨勫Э鎬佽绠椾袱涓叧閿抚涔嬮棿鐨勫熀鏈煩闃�
 * @param  pKF1 鍏抽敭甯�1
 * @param  pKF2 鍏抽敭甯�2
 * @return      鍩烘湰鐭╅樀
 */
cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    // Essential Matrix: t12鍙変箻R12
    // Fundamental Matrix: inv(K1)*E*inv(K2)
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}
/**
 * @brief 鍏抽敭甯у墧闄�
 * 
 * 鍦–ovisibility Graph涓殑鍏抽敭甯э紝鍏�90%浠ヤ笂鐨凪apPoints鑳借鍏朵粬鍏抽敭甯э紙鑷冲皯3涓級瑙傛祴鍒帮紝鍒欒涓鸿鍏抽敭甯т负鍐椾綑鍏抽敭甯с€�
 * @see VI-E Local Keyframe Culling
 */
void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    // 姝ラ1锛氭牴鎹瓹ovisibility Graph鎻愬彇褰撳墠甯х殑鍏辫鍏抽敭甯�
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    // 瀵规墍鏈夌殑灞€閮ㄥ叧閿抚杩涜閬嶅巻
    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        // 姝ラ2锛氭彁鍙栨瘡涓叡瑙嗗叧閿抚鐨凪apPoints
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        // 姝ラ3锛氶亶鍘嗚灞€閮ㄥ叧閿抚鐨凪apPoints锛屽垽鏂槸鍚�90%浠ヤ笂鐨凪apPoints鑳借鍏跺畠鍏抽敭甯э紙鑷冲皯3涓級瑙傛祴鍒�
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        // 瀵逛簬鍙岀洰锛屼粎鑰冭檻杩戝鐨凪apPoints锛屼笉瓒呰繃mbf * 35 / fx
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    // MapPoints鑷冲皯琚笁涓叧閿抚瑙傛祴鍒�
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        // 鍒ゆ柇璇apPoint鏄惁鍚屾椂琚笁涓叧閿抚瑙傛祴鍒�
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            // Scale Condition 
                            // 灏哄害绾︽潫锛岃姹侻apPoint鍦ㄨ灞€閮ㄥ叧閿抚鐨勭壒寰佸昂搴﹀ぇ浜庯紙鎴栬繎浼间簬锛夊叾瀹冨叧閿抚鐨勭壒寰佸昂搴�
                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                // 宸茬粡鎵惧埌涓変釜鍚屽昂搴︾殑鍏抽敭甯у彲浠ヨ娴嬪埌璇apPoint锛屼笉鐢ㄧ户缁壘浜�
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        // 璇apPoint鑷冲皯琚笁涓叧閿抚瑙傛祴鍒�
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  

        // 姝ラ4锛氳灞€閮ㄥ叧閿抚90%浠ヤ笂鐨凪apPoints鑳借鍏跺畠鍏抽敭甯э紙鑷冲皯3涓級瑙傛祴鍒帮紝鍒欒涓烘槸鍐椾綑鍏抽敭甯�
        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
