/**
* Used to store extracted keyframes
*/

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc)
{
    mvInvertedFile.resize(voc.size());
}


/**
 * @brief 根据关键帧的词包，更新数据库的��排索引
 * @param pKF 关键帄1�7
 */
void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    // 为每丢�个word添加该KeyFrame
    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}
/**
 * @brief 关键帧被删除后，更新数据库的倒排索引
 * @param pKF 关键帄1�7
 */
void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    // 每一个KeyFrame包含多个words，遍历mvInvertedFile中的这些words，然后在word中删除该KeyFrame
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

/**
 * @brief 在闭环检测中找到与该关键帧可能闭环的关键帄1�7
 *
 * 1. 找出和当前帧具有公共单词的所有关键帧（不包括与当前帧相连的关键帧＄1�7
 * 2. 只和具有共同单词较多的关键帧进行相似度计箄1�7
 * 3. 将与关键帧相连（权��最高）的前十个关键帧归为一组，计算累计得分
 * 4. 只返回累计得分较高的组中分数朢�高的关键帄1�7
 * @param pKF      霢�要闭环的关键帄1�7
 * @param minScore 相似性分数最低要汄1�7
 * @return         可能闭环的关键帧
 * @see III-E Bags of Words Place Recognition
 */
vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)
{
    // 提出扢�有与该pKF相连的KeyFrame，这些相连Keyframe都是屢�部相连，在闭环检测的时��将被剔附1�7
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<KeyFrame*> lKFsSharingWords;// 用于保存可能与pKF形成回环的����帧（只要有相同的word，且不属于局部相连帧＄1�7

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    // 步骤1：找出和当前帧具有公共单词的扢�有关键帧（不包括与当前帧链接的关键帧＄1�7
    {
        unique_lock<mutex> lock(mMutex);

        // words是检测图像是否匹配的枢纽，遍历该pKF的每丢�个word
        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)//访问当前帧的单词
        {
            // 提取扢�有包含该word的KeyFrame
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnLoopQuery!=pKF->mnId)// 判断该帧是否为pKF的����帧
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi))// 与pKF屢�部链接的关键帧不进入闭环候��帧
                    {
                        pKFi->mnLoopQuery=pKF->mnId;// pKFi标记为pKF的����帧，之后直接跳过判斄1�7
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;// 记录pKFi与pKF具有相同word的个敄1�7
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    // 步骤2：统计所有闭环����帧中与pKF具有共同单词朢�多的单词敄1�7
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    // 步骤3：遍历所有闭环����帧，挑选出共有单词数大于minCommonWords且单词匹配度大于minScore存入lScoreAndMatch
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        // pKF只和具有共同单词较多的关键帧进行比较，需要大于minCommonWords
        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧相连（权��最高，共视程度朢�高）的前十个关键帧归为一组，计算累计得分
    // 步骤4：具体��言：lScoreAndMatch中每丢�个KeyFrame都把与自己共视程度较高的帧归为一组，每一组会计算组得分并记录该组分数朢�高的KeyFrame，记录于lAccScoreAndMatch
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);//取与该帧共视的1�710个帧存入 

        float bestScore = it->first;// 该组朢�高分敄1�7
        float accScore = it->first;// 该组累计得分
        KeyFrame* pBestKF = pKFi;// 该组朢�高分数对应的关键帄1�7
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;// 因为pKF2->mnLoopQuery==pKF->mnId，所以只有pKF2也在闭环候��帧中，才能贡献分数
                if(pKF2->mLoopScore>bestScore)// 统计得到组里分数朢�高的KeyFrame
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)// 记录扢�有组中组得分朢�高的组1�7
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    // 步骤5：得到组得分大于minScoreToRetain的组，得到组中分数最高的关键帄1�7 0.75*bestScore
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))// 判断该pKFi是否已经在队列中亄1�7
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

/**
 * @brief 在重定位中找到与该帧相似的关键帧
 *
 * 1. 找出和当前帧具有公共单词的所有关键帧
 * 2. 只和具有共同单词较多的关键帧进行相似度计箄1�7
 * 3. 将与关键帧相连（权��最高）的前十个关键帧归为一组，计算累计得分
 * 4. 只返回累计得分较高的组中分数朢�高的关键帄1�7
 * @param F 霢�要重定位的帧
 * @return  相似的关键帧
 * @see III-E Bags of Words Place Recognition
 */
vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
    // 相对于关键帧的闭环检测DetectLoopCandidates，重定位棢�测中没法获得相连的关键帧
    list<KeyFrame*> lKFsSharingWords;// 用于保存可能与F形成回环的����帧（只要有相同的word，且不属于局部相连帧＄1�7

    // Search all keyframes that share a word with current frame
    // 步骤1：找出和当前帧具有公共单词的扢�有关键帧
    {
        unique_lock<mutex> lock(mMutex);

        // words是检测图像是否匹配的枢纽，遍历该pKF的每丢�个word
        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            // 提取扢�有包含该word的KeyFrame
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnRelocQuery!=F->mnId)// pKFi还没有标记为pKF的����帧
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    // Only compare against those keyframes that share enough words
    // 步骤2：统计所有闭环����帧中与当前帧F具有共同单词朢�多的单词数，并以此决定阈倄1�7
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    // 步骤3：遍历所有闭环����帧，挑选出共有单词数大于阈值minCommonWords且单词匹配度大于minScore存入lScoreAndMatch
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        // 当前帧F只和具有共同单词较多的关键帧进行比较，需要大于minCommonWords
        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;// 这个变量后面没有用到
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    // 步骤4：计算����帧组得分，得到朢�高组得分bestAccScore，并以此决定阈��minScoreToRetain
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧相连（权��最高，共视程度朢�高）的前十个关键帧归为一组，计算累计得分
    // 具体而言：lScoreAndMatch中每丢�个KeyFrame都把与自己共视程度较高的帧归为一组，每一组会计算组得分并记录该组分数朢�高的KeyFrame，记录于lAccScoreAndMatch
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;// 该组朢�高分敄1�7
        float accScore = bestScore;// 该组累计得分
        KeyFrame* pBestKF = pKFi;// 该组朢�高分数对应的关键帄1�7
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;

            accScore+=pKF2->mRelocScore;// 只有pKF2也在闭环候��帧中，才能贡献分数
            if(pKF2->mRelocScore>bestScore)// 统计得到组里分数朢�高的KeyFrame
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)// 记录扢�有组中组得分朢�高的组1�7
            bestAccScore=accScore;// 得到扢�有组中最高的累计得分
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    // 步骤5：得到组得分大于阈��的，组内得分最高的关键帄1�7
    float minScoreToRetain = 0.75f*bestAccScore;
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        // 只返回累计得分大于minScoreToRetain的组中分数最高的关键帄1�7 0.75*bestScore
        if(si>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))// 判断该pKFi是否已经在队列中亄1�7
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

} //namespace ORB_SLAM
