/**
* Visual odometer initialization
*/

#include "Initializer.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include "Optimizer.h"
#include "ORBmatcher.h"

#include<thread>

namespace ORB_SLAM2
{
/**
 * @brief 缁欏畾鍙傝€冨抚鏋勯€營nitializer
 * 
 * 鐢╮eference frame鏉ュ垵濮嬪寲锛岃繖涓猺eference frame灏辨槸SLAM姝ｅ紡寮€濮嬬殑绗竴甯�
 * @param ReferenceFrame 鍙傝€冨抚
 * @param sigma          娴嬮噺璇樊
 * @param iterations     RANSAC杩唬娆℃暟
 */
Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
{
    mK = ReferenceFrame.mK.clone();

    mvKeys1 = ReferenceFrame.mvKeysUn;

    mSigma = sigma;
    mSigma2 = sigma*sigma;
    mMaxIterations = iterations;
}
/**
 * @brief 骞惰鍦拌绠楀熀纭€鐭╅樀鍜屽崟搴旀€х煩闃碉紝閫夊彇鍏朵腑涓€涓ā鍨嬶紝鎭㈠鍑烘渶寮€濮嬩袱甯т箣闂寸殑鐩稿濮挎€佷互鍙婄偣浜�
 */
bool Initializer::Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                             vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
{
    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    // Frame2 鐗瑰緛鐐�
    mvKeys2 = CurrentFrame.mvKeysUn;

    // mvMatches12璁板綍鍖归厤涓婄殑鐗瑰緛鐐瑰
    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    // mvbMatched1璁板綍姣忎釜鐗瑰緛鐐规槸鍚︽湁鍖归厤鐨勭壒寰佺偣锛�
    // 杩欎釜鍙橀噺鍚庨潰娌℃湁鐢ㄥ埌锛屽悗闈㈠彧鍏冲績鍖归厤涓婄殑鐗瑰緛鐐�
    mvbMatched1.resize(mvKeys1.size());

    // 姝ラ1锛氱粍缁囩壒寰佺偣瀵�
    for(size_t i=0, iend=vMatches12.size();i<iend; i++)
    {
        if(vMatches12[i]>=0)
        {
            mvMatches12.push_back(make_pair(i,vMatches12[i]));
            mvbMatched1[i]=true;
        }
        else
            mvbMatched1[i]=false;
    }

    // 鍖归厤涓婄殑鐗瑰緛鐐圭殑涓暟
    const int N = mvMatches12.size();

    // Indices for minimum set selection
    // 鏂板缓涓€涓鍣╲AllIndices锛岀敓鎴�0鍒癗-1鐨勬暟浣滀负鐗瑰緛鐐圭殑绱㈠紩
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    // 姝ラ2锛氬湪鎵€鏈夊尮閰嶇壒寰佺偣瀵逛腑闅忔満閫夋嫨8瀵瑰尮閰嶇壒寰佺偣涓轰竴缁勶紝鍏遍€夋嫨mMaxIterations缁�
    // 鐢ㄤ簬FindHomography鍜孎indFundamental姹傝В
    // mMaxIterations:200
    mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));

    DUtils::Random::SeedRandOnce(0);

    for(int it=0; it<mMaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            // 浜х敓0鍒癗-1鐨勯殢鏈烘暟
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            // idx琛ㄧず鍝竴涓储寮曞搴旂殑鐗瑰緛鐐硅閫変腑
            int idx = vAvailableIndices[randi];

            mvSets[it][j] = idx;


            // randi瀵瑰簲鐨勭储寮曞凡缁忚閫夎繃浜嗭紝浠庡鍣ㄤ腑鍒犻櫎
            // randi瀵瑰簲鐨勭储寮曠敤鏈€鍚庝竴涓厓绱犳浛鎹紝骞跺垹鎺夋渶鍚庝竴涓厓绱�
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    // 姝ラ3锛氳皟鐢ㄥ绾跨▼鍒嗗埆鐢ㄤ簬璁＄畻fundamental matrix鍜宧omography
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    float SH, SF;// score for H and F
    cv::Mat H, F;// H and F

    // ref鏄紩鐢ㄧ殑鍔熻兘:http://en.cppreference.com/w/cpp/utility/functional/ref
    // 璁＄畻homograpy骞舵墦鍒�
    thread threadH(&Initializer::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
    // 璁＄畻fundamental matrix骞舵墦鍒�
    thread threadF(&Initializer::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    // 姝ラ4锛氳绠楀緱鍒嗘瘮渚嬶紝閫夊彇鏌愪釜妯″瀷
    float RH = SH/(SH+SF);

    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    // 姝ラ5锛氫粠H鐭╅樀鎴朏鐭╅樀涓仮澶峈,t
    if(RH>0.40)
        return ReconstructH(vbMatchesInliersH,H,mK,R21,t21,vP3D,vbTriangulated,1.0,50);
    else //if(pF_HF>0.6)
        return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,1.0,50);

    return false;
}

/**
 * @brief 璁＄畻鍗曞簲鐭╅樀
 *
 * 鍋囪鍦烘櫙涓哄钩闈㈡儏鍐典笅閫氳繃鍓嶄袱甯ф眰鍙朒omography鐭╅樀(current frame 2 鍒� reference frame 1),骞跺緱鍒拌妯″瀷鐨勮瘎鍒�
 */
void Initializer::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
{
    // Number of putative matches
    const int N = mvMatches12.size();

    // Normalize coordinates
    // 灏唌vKeys1鍜宮vKey2褰掍竴鍖栧埌鍧囧€间负0锛屼竴闃剁粷瀵圭煩涓�1锛屽綊涓€鍖栫煩闃靛垎鍒负T1銆乀2
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2inv = T2.inv();

    // Best Results variables
    // 鏈€缁堟渶浣崇殑MatchesInliers涓庡緱鍒�
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat H21i, H12i;
    // 姣忔RANSAC鐨凪atchesInliers涓庡緱鍒�
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            // vPn1i鍜寁Pn2i涓哄尮閰嶇殑鐗瑰緛鐐瑰鐨勫潗鏍�
            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Hn = ComputeH21(vPn1i,vPn2i);
        // 鎭㈠鍘熷鐨勫潎鍊煎拰灏哄害
        H21i = T2inv*Hn*T1;
        H12i = H21i.inv();

        // 鍒╃敤閲嶆姇褰辫宸负褰撴RANSAC鐨勭粨鏋滆瘎鍒�
        currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

        // 寰楀埌鏈€浼樼殑vbMatchesInliers涓巗core
        if(currentScore>score)
        {
            H21 = H21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

/**
 * @brief 璁＄畻鍩虹鐭╅樀
 *
 * 鍋囪鍦烘櫙涓洪潪骞抽潰鎯呭喌涓嬮€氳繃鍓嶄袱甯ф眰鍙朏undamental鐭╅樀(current frame 2 鍒� reference frame 1),骞跺緱鍒拌妯″瀷鐨勮瘎鍒�
 */
void Initializer::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
{
    // Number of putative matches
    const int N = vbMatchesInliers.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2t = T2.t();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat F21i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(int j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Fn = ComputeF21(vPn1i,vPn2i);

        F21i = T2t*Fn*T1;

        // 鍒╃敤閲嶆姇褰辫宸负褰撴RANSAC鐨勭粨鏋滆瘎鍒�
        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

        if(currentScore>score)
        {
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

// |x'|     | h1 h2 h3 ||x|
// |y'| = a | h4 h5 h6 ||y|  绠€鍐�: x' = a H x, a涓轰竴涓昂搴﹀洜瀛�
// |1 |     | h7 h8 h9 ||1|
// 浣跨敤DLT(direct linear tranform)姹傝В璇ユā鍨�
// x' = a H x 
// ---> (x') 鍙変箻 (H x)  = 0
// ---> Ah = 0
// A = | 0  0  0 -x -y -1 xy' yy' y'|  h = | h1 h2 h3 h4 h5 h6 h7 h8 h9 |
//     |-x -y -1  0  0  0 xx' yx' x'|
// 閫氳繃SVD姹傝ВAh = 0锛孉'A鏈€灏忕壒寰佸€煎搴旂殑鐗瑰緛鍚戦噺鍗充负瑙�

/**
 * @brief 浠庣壒寰佺偣鍖归厤姹俬omography锛坣ormalized DLT锛�
 * 
 * @param  vP1 褰掍竴鍖栧悗鐨勭偣, in reference frame
 * @param  vP2 褰掍竴鍖栧悗鐨勭偣, in current frame
 * @return     鍗曞簲鐭╅樀
 * @see        Multiple View Geometry in Computer Vision - Algorithm 4.2 p109
 */
cv::Mat Initializer::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(2*N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(2*i,0) = 0.0;
        A.at<float>(2*i,1) = 0.0;
        A.at<float>(2*i,2) = 0.0;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1;
        A.at<float>(2*i+1,3) = 0.0;
        A.at<float>(2*i+1,4) = 0.0;
        A.at<float>(2*i+1,5) = 0.0;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;

    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    return vt.row(8).reshape(0, 3);// v鐨勬渶鍚庝竴鍒�
}
// x'Fx = 0 鏁寸悊鍙緱锛欰f = 0
// A = | x'x x'y x' y'x y'y y' x y 1 |, f = | f1 f2 f3 f4 f5 f6 f7 f8 f9 |
// 閫氳繃SVD姹傝ВAf = 0锛孉'A鏈€灏忕壒寰佸€煎搴旂殑鐗瑰緛鍚戦噺鍗充负瑙�

/**
 * @brief 浠庣壒寰佺偣鍖归厤姹俧undamental matrix锛坣ormalized 8鐐规硶锛�
 * @param  vP1 褰掍竴鍖栧悗鐨勭偣, in reference frame
 * @param  vP2 褰掍竴鍖栧悗鐨勭偣, in current frame
 * @return     鍩虹鐭╅樀
 * @see        Multiple View Geometry in Computer Vision - Algorithm 11.1 p282 (涓枃鐗� p191)
 */
cv::Mat Initializer::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(N,9,CV_32F);// N*9

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);// v鐨勬渶鍚庝竴鍒�

    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2)=0;// 绉�2绾︽潫锛屽皢绗�3涓寮傚€艰涓�0

    return  u*cv::Mat::diag(w)*vt;
}
/**
 * @brief 瀵圭粰瀹氱殑homography matrix鎵撳垎
 * 
 * @see
 * - Author's paper - IV. AUTOMATIC MAP INITIALIZATION 锛�2锛�
 * - Multiple View Geometry in Computer Vision - symmetric transfer errors: 4.2.2 Geometric distance
 * - Multiple View Geometry in Computer Vision - model selection 4.7.1 RANSAC
 */
float Initializer::CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma)
{   
    const int N = mvMatches12.size();

    // |h11 h12 h13|
    // |h21 h22 h23|
    // |h31 h32 h33|
    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);

    // |h11inv h12inv h13inv|
    // |h21inv h22inv h23inv|
    // |h31inv h32inv h33inv|
    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;
    // 鍩轰簬鍗℃柟妫€楠岃绠楀嚭鐨勯槇鍊硷紙鍋囪娴嬮噺鏈変竴涓儚绱犵殑鍋忓樊锛�
    const float th = 5.991;

    //淇℃伅鐭╅樀锛屾柟宸钩鏂圭殑鍊掓暟
    const float invSigmaSquare = 1.0/(sigma*sigma);

    // N瀵圭壒寰佸尮閰嶇偣
    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in first image
        // x2in1 = H12*x2
        // 灏嗗浘鍍�2涓殑鐗瑰緛鐐瑰崟搴斿埌鍥惧儚1涓�
        // |u1|   |h11inv h12inv h13inv||u2|
        // |v1| = |h21inv h22inv h23inv||v2|
        // |1 |   |h31inv h32inv h33inv||1 |
        const float w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
        const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
        const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

        // 璁＄畻閲嶆姇褰辫宸�
        const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

        // 鏍规嵁鏂瑰樊褰掍竴鍖栬宸�
        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = H21*x1
        // 灏嗗浘鍍�1涓殑鐗瑰緛鐐瑰崟搴斿埌鍥惧儚2涓�
        const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
        const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += th - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}
/**
 * @brief 瀵圭粰瀹氱殑fundamental matrix鎵撳垎
 * 
 * @see
 * - Author's paper - IV. AUTOMATIC MAP INITIALIZATION 锛�2锛�
 * - Multiple View Geometry in Computer Vision - symmetric transfer errors: 4.2.2 Geometric distance
 * - Multiple View Geometry in Computer Vision - model selection 4.7.1 RANSAC
 */
float Initializer::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    // 鍩轰簬鍗℃柟妫€楠岃绠楀嚭鐨勯槇鍊硷紙鍋囪娴嬮噺鏈変竴涓儚绱犵殑鍋忓樊锛�
    const float th = 3.841;
    const float thScore = 5.991;

    const float invSigmaSquare = 1.0/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)
        // F21x1鍙互绠楀嚭x1鍦ㄥ浘鍍忎腑x2瀵瑰簲鐨勭嚎l
        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;

        // x2搴旇鍦╨杩欐潯绾夸笂:x2鐐逛箻l = 0 
        const float num2 = a2*u2+b2*v2+c2;

        const float squareDist1 = num2*num2/(a2*a2+b2*b2);// 鐐瑰埌绾跨殑鍑犱綍璺濈 鐨勫钩鏂�

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)

        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        const float num1 = a1*u1+b1*v1+c1;

        const float squareDist2 = num1*num1/(a1*a1+b1*b1);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}
//                          |0 -1  0|
// E = U Sigma V'   let W = |1  0  0|
//                          |0  0  1|
// 寰楀埌4涓В E = [R|t]
// R1 = UWV' R2 = UW'V' t1 = U3 t2 = -U3

/**
 * @brief 浠嶧鎭㈠R t
 * 
 * 搴﹂噺閲嶆瀯
 * 1. 鐢盕undamental鐭╅樀缁撳悎鐩告満鍐呭弬K锛屽緱鍒癊ssential鐭╅樀: \f$ E = k'^T F k \f$
 * 2. SVD鍒嗚В寰楀埌R t
 * 3. 杩涜cheirality check, 浠庡洓涓В涓壘鍑烘渶鍚堥€傜殑瑙�
 * 
 * @see Multiple View Geometry in Computer Vision - Result 9.19 p259
 */
bool Initializer::ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                            cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat E21 = K.t()*F21*K;

    cv::Mat R1, R2, t;

    // Recover the 4 motion hypotheses
    // 铏界劧杩欎釜鍑芥暟瀵箃鏈夊綊涓€鍖栵紝浣嗗苟娌℃湁鍐冲畾鍗曠洰鏁翠釜SLAM杩囩▼鐨勫昂搴�
    // 鍥犱负CreateInitialMapMonocular鍑芥暟瀵�3D鐐规繁搴︿細缂╂斁锛岀劧鍚庡弽杩囨潵瀵� t 鏈夋敼鍙�
    DecomposeE(E21,R1,R2,t);  

    cv::Mat t1=t;
    cv::Mat t2=-t;

    // Reconstruct with the 4 hyphoteses and check

    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
    float parallax1,parallax2, parallax3, parallax4;

    int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();
    // minTriangulated涓哄彲浠ヤ笁瑙掑寲鎭㈠涓夌淮鐐圭殑涓暟
    int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

    int nsimilar = 0;
    if(nGood1>0.7*maxGood)
        nsimilar++;
    if(nGood2>0.7*maxGood)
        nsimilar++;
    if(nGood3>0.7*maxGood)
        nsimilar++;
    if(nGood4>0.7*maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    // 鍥涗釜缁撴灉涓鏋滄病鏈夋槑鏄剧殑鏈€浼樼粨鏋滐紝鍒欒繑鍥炲け璐�
    if(maxGood<nMinGood || nsimilar>1)
    {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    // 姣旇緝澶х殑瑙嗗樊瑙�
    if(maxGood==nGood1)
    {
        if(parallax1>minParallax)
        {
            vP3D = vP3D1;
            vbTriangulated = vbTriangulated1;

            R1.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2>minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood3)
    {
        if(parallax3>minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood4)
    {
        if(parallax4>minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }

    return false;
}
// H鐭╅樀鍒嗚В甯歌鏈変袱绉嶆柟娉曪細Faugeras SVD-based decomposition 鍜� Zhang SVD-based decomposition
// 鍙傝€冩枃鐚細Motion and structure from motion in a piecewise plannar environment
// 杩欑瘒鍙傝€冩枃鐚拰涓嬮潰鐨勪唬鐮佷娇鐢ㄤ簡Faugeras SVD-based decomposition绠楁硶

/**
 * @brief 浠嶩鎭㈠R t
 *
 * @see
 * - Faugeras et al, Motion and structure from motion in a piecewise planar environment. International Journal of Pattern Recognition and Artificial Intelligence, 1988.
 * - Deeper understanding of the homography decomposition for vision-based control
 */
bool Initializer::ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988

    // 鍥犱负鐗瑰緛鐐规槸鍥惧儚鍧愭爣绯伙紝鎵€浠ヨH鐭╅樀鐢辩浉鏈哄潗鏍囩郴鎹㈢畻鍒板浘鍍忓潗鏍囩郴
    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;

    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    // SVD鍒嗚В鐨勬甯告儏鍐垫槸鐗瑰緛鍊奸檷搴忔帓鍒�
    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
        return false;
    }

    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    // 娉曞悜閲弉'= [x1 0 x3] 瀵瑰簲ppt鐨勫叕寮�17
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    // 璁＄畻ppt涓叕寮�19
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    // 璁＄畻鏃嬭浆鐭╅樀 R鈥橈紝璁＄畻ppt涓叕寮�18
    //         | ctheta         0      -aux_stheta|          | aux1|
    // Rp = |    0             1       0                |  tp = |  0     |
    //         | aux_stheta  0      ctheta         |         |-aux3|

    //         | ctheta         0    aux_stheta   |         | aux1|
    // Rp = |    0              1       0              |  tp = |  0     |
    //         |-aux_stheta  0    ctheta          |         | aux3|

    //         | ctheta         0    aux_stheta|         |-aux1|
    // Rp = |    0              1       0           |  tp = |  0     |
    //         |-aux_stheta  0    ctheta       |         |-aux3|

    //         | ctheta         0   -aux_stheta|         |-aux1|
    // Rp = |    0              1       0           |  tp = |  0     |
    //         | aux_stheta  0    ctheta       |         | aux3|
    for(int i=0; i<4; i++)  
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;
        // 杩欓噷铏界劧瀵箃鏈夊綊涓€鍖栵紝骞舵病鏈夊喅瀹氬崟鐩暣涓猄LAM杩囩▼鐨勫昂搴�
        // 鍥犱负CreateInitialMapMonocular鍑芥暟瀵�3D鐐规繁搴︿細缂╂斁锛岀劧鍚庡弽杩囨潵瀵� t 鏈夋敼鍙�
        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    //case d'=-d2
    // 璁＄畻ppt涓叕寮�22
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    // 璁＄畻鏃嬭浆鐭╅樀 R鈥橈紝璁＄畻ppt涓叕寮�21
    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }


    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<cv::Point3f> bestP3D;
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    // d'=d2鍜宒'=-d2鍒嗗埆瀵瑰簲8缁�(R t)
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);

        // 淇濈暀鏈€浼樼殑鍜屾浼樼殑
        if(nGood>bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
        }
    }


    if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D = bestP3D;
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}
// Trianularization: 宸茬煡鍖归厤鐗瑰緛鐐瑰{x x'} 鍜� 鍚勮嚜鐩告満鐭╅樀{P P'}, 浼拌涓夌淮鐐� X
// x' = P'X  x = PX
// 瀹冧滑閮藉睘浜� x = aPX妯″瀷
//                         |X|
// |x|       |p1 p2  p3  p4    |   |Y|         |x|      |--p0--|  |.|
// |y| = a |p5 p6  p7  p8    |   |Z| ===>|y| = a |--p1--|  |X|
// |z|       |p9 p10 p11 p12|   |1|         |z|      |--p2--|   |.|
// 閲囩敤DLT鐨勬柟娉曪細x鍙変箻PX = 0
// |yp2 -  p1|       |0|
// |p0 -  xp2| X = |0|
// |xp1 - yp0|      |0|
// 涓や釜鐐�:
// |yp2   -  p1  |       |0|
// |p0    -  xp2 | X = |0| ===> AX = 0
// |y'p2' -  p1' |        |0|
// |p0'   - x'p2'|        |0|
// 鍙樻垚绋嬪簭涓殑褰㈠紡锛�
// |xp2  - p0 |       |0|
// |yp2  - p1 | X = |0| ===> AX = 0
// |x'p2'- p0'|        |0|
// |y'p2'- p1'|        |0|

/**
 * @brief 缁欏畾鎶曞奖鐭╅樀P1,P2鍜屽浘鍍忎笂鐨勭偣kp1,kp2锛屼粠鑰屾仮澶�3D鍧愭爣
 *
 * @param kp1 鐗瑰緛鐐�, in reference frame
 * @param kp2 鐗瑰緛鐐�, in current frame
 * @param P1  鎶曞奖鐭╅樀P1
 * @param P2  鎶曞奖鐭╅樀P锛�
 * @param x3D 涓夌淮鐐�
 * @see       Multiple View Geometry in Computer Vision - 12.2 Linear triangulation methods p312
 */
void Initializer::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    // 鍦―ecomposeE鍑芥暟鍜孯econstructH鍑芥暟涓t鏈夊綊涓€鍖�
    // 杩欓噷涓夎鍖栬繃绋嬩腑鎭㈠鐨�3D鐐规繁搴﹀彇鍐充簬 t 鐨勫昂搴︼紝
    // 浣嗘槸杩欓噷鎭㈠鐨�3D鐐瑰苟娌℃湁鍐冲畾鍗曠洰鏁翠釜SLAM杩囩▼鐨勫昂搴�
    // 鍥犱负CreateInitialMapMonocular鍑芥暟瀵�3D鐐规繁搴︿細缂╂斁锛岀劧鍚庡弽杩囨潵瀵� t 鏈夋敼鍙�
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}
/**
 * 锛燽rief 褰掍竴鍖栫壒寰佺偣鍒板悓涓€灏哄害锛堜綔涓簄ormalize DLT鐨勮緭鍏ワ級
 *
 * [x' y' 1]' = T * [x y 1]' \n
 * 褰掍竴鍖栧悗x', y'鐨勫潎鍊间负0锛宻um(abs(x_i'-0))=1锛宻um(abs((y_i'-0))=1
 * 
 * @param vKeys             鐗瑰緛鐐瑰湪鍥惧儚涓婄殑鍧愭爣
 * @param vNormalizedPoints 鐗瑰緛鐐瑰綊涓€鍖栧悗鐨勫潗鏍�
 * @param T                 灏嗙壒寰佺偣褰掍竴鍖栫殑鐭╅樀
 */
void Initializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    meanX = meanX/N;
    meanY = meanY/N;

    float meanDevX = 0;
    float meanDevY = 0;

    // 灏嗘墍鏈塿Keys鐐瑰噺鍘讳腑蹇冨潗鏍囷紝浣縳鍧愭爣鍜寉鍧愭爣鍧囧€煎垎鍒负0
    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;

    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;

    // 灏唜鍧愭爣鍜寉鍧愭爣鍒嗗埆杩涜灏哄害缂╂斁锛屼娇寰梮鍧愭爣鍜寉鍧愭爣鐨勪竴闃剁粷瀵圭煩鍒嗗埆涓�1
    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }


    // |sX  0  -meanx*sX|
    // |0   sY -meany*sY|
    // |0   0      1    |
    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}

/**
 * @brief 杩涜cheirality check锛屼粠鑰岃繘涓€姝ユ壘鍑篎鍒嗚В鍚庢渶鍚堥€傜殑瑙�
 */
int Initializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    // Calibration parameters
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    // 姝ラ1锛氬緱鍒颁竴涓浉鏈虹殑鎶曞奖鐭╅樀
    // 浠ョ涓€涓浉鏈虹殑鍏夊績浣滀负涓栫晫鍧愭爣绯�
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));
    // 绗竴涓浉鏈虹殑鍏夊績鍦ㄤ笘鐣屽潗鏍囩郴涓嬬殑鍧愭爣
    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    // 姝ラ2锛氬緱鍒扮浜屼釜鐩告満鐨勬姇褰辩煩闃�
    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;
    // 绗簩涓浉鏈虹殑鍏夊績鍦ㄤ笘鐣屽潗鏍囩郴涓嬬殑鍧愭爣
    cv::Mat O2 = -R.t()*t;

    int nGood=0;

    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        if(!vbMatchesInliers[i])
            continue;

        // kp1鍜宬p2鏄尮閰嶇壒寰佺偣
        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
        cv::Mat p3dC1;

        // 姝ラ3锛氬埄鐢ㄤ笁瑙掓硶鎭㈠涓夌淮鐐筽3dC1
        Triangulate(kp1,kp2,P1,P2,p3dC1);

        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first]=false;
            continue;
        }

        // Check parallax
        // 姝ラ4锛氳绠楄宸浣欏鸡鍊�
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);

        float cosParallax = normal1.dot(normal2)/(dist1*dist2);

        // 姝ラ5锛氬垽鏂�3D鐐规槸鍚﹀湪涓や釜鎽勫儚澶村墠鏂�
        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        // 姝ラ5.1锛�3D鐐规繁搴︿负璐燂紝鍦ㄧ涓€涓憚鍍忓ご鍚庢柟锛屾窐姹�
        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        // 姝ラ5.2锛�3D鐐规繁搴︿负璐燂紝鍦ㄧ浜屼釜鎽勫儚澶村悗鏂癸紝娣樻卑
        cv::Mat p3dC2 = R*p3dC1+t;

        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // 姝ラ6锛氳绠楅噸鎶曞奖璇樊
        // Check reprojection error in first image
        // 璁＄畻3D鐐瑰湪绗竴涓浘鍍忎笂鐨勬姇褰辫宸�
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        // 姝ラ6.1锛氶噸鎶曞奖璇樊澶ぇ锛岃烦杩囨窐姹�
        // 涓€鑸宸姣旇緝灏忔椂閲嶆姇褰辫宸瘮杈冨ぇ
        if(squareError1>th2)
            continue;

        // Check reprojection error in second image
        // 璁＄畻3D鐐瑰湪绗簩涓浘鍍忎笂鐨勬姇褰辫宸�
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        // 姝ラ6.2锛氶噸鎶曞奖璇樊澶ぇ锛岃烦杩囨窐姹�
        // 涓€鑸宸姣旇緝灏忔椂閲嶆姇褰辫宸瘮杈冨ぇ
        if(squareError2>th2)
            continue;

        // 姝ラ7锛氱粺璁＄粡杩囨楠岀殑3D鐐逛釜鏁帮紝璁板綍3D鐐硅宸
        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;

        if(cosParallax<0.99998)
            vbGood[vMatches12[i].first]=true;
    }

    // 姝ラ8锛氬緱鍒�3D鐐逛腑杈冨ぇ鐨勮宸
    if(nGood>0)
    {
        // 浠庡皬鍒板ぇ鎺掑簭
        sort(vCosParallax.begin(),vCosParallax.end());
       // trick! 鎺掑簭鍚庡苟娌℃湁鍙栨渶澶х殑瑙嗗樊瑙�
        // 鍙栦竴涓緝澶х殑瑙嗗樊瑙�
        size_t idx = min(50,int(vCosParallax.size()-1));
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}
/**
 * @brief 鍒嗚ВEssential鐭╅樀
 * 
 * F鐭╅樀閫氳繃缁撳悎鍐呭弬鍙互寰楀埌Essential鐭╅樀锛屽垎瑙鐭╅樀灏嗗緱鍒�4缁勮В \n
 * 杩�4缁勮В鍒嗗埆涓篬R1,t],[R1,-t],[R2,t],[R2,-t]
 * @param E  Essential Matrix
 * @param R1 Rotation Matrix 1
 * @param R2 Rotation Matrix 2
 * @param t  Translation
 * @see Multiple View Geometry in Computer Vision - Result 9.19 p259
 */
void Initializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);
    // 瀵� t 鏈夊綊涓€鍖栵紝浣嗘槸杩欎釜鍦版柟骞舵病鏈夊喅瀹氬崟鐩暣涓猄LAM杩囩▼鐨勫昂搴�
    // 鍥犱负CreateInitialMapMonocular鍑芥暟瀵�3D鐐规繁搴︿細缂╂斁锛岀劧鍚庡弽杩囨潵瀵� t 鏈夋敼鍙�
    u.col(2).copyTo(t);
    t=t/cv::norm(t);

    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

    R1 = u*W*vt;
    if(cv::determinant(R1)<0)// 鏃嬭浆鐭╅樀鏈夎鍒楀紡涓�1鐨勭害鏉�
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
}

} //namespace ORB_SLAM
