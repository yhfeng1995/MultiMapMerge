#include "MultiMapMerge/KeyFrameNode.h"

namespace MultiMapMerge
{
    BoWScoreT KeyFrameNode::SIMILAR_VAL_THRES = 5;

    KeyFrameNode::KeyFrameNode(ORB_SLAM2::KeyFrame *pKf, PointCloudT::Ptr pKfCloud):
    mpKf(pKf), mpKfCloud(pKfCloud) {}


    /// 计算关键帧的相似度
    BoWScoreT KeyFrameNode::computeSimiliarity(const KeyFrameNode::Ptr &rhs) {
        // TODO: ...
    }


    /// 关键帧位姿转移坐标系
    bool KeyFrameNode::transformByPose(const cv::Mat &Twr) {
        // TODO: ...
    }


    /// 输出变换点云
    bool KeyFrameNode::getTransformedCloud(PointCloudT::Ptr &dstCloud) {
        // TODO: ...
    }

} // namespace MultiMapMerge
