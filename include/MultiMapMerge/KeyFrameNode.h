/**
 * @file KeyFrameNode.h
 * @author Yonghui Feng (yhfeng1995@gmail.com)
 * @brief 单地图基本数据结构单位定义
 * @version 0.1
 * @date 2020-11-05
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef MULTIMAPMERGE_KEYFRAMENODE_H
#define MULTIMAPMERGE_KEYFRAMENODE_H

#include "MultiMapMerge/common.h"

namespace ORB_SLAM2 {
    class KeyFrame;
};

namespace MultiMapMerge {
    /**
     * @brief 关键帧与对应点云组合结构, DiscreteGlobalMap的
     * 基本组成单位
     */
    struct KeyFrameNode {
    public:
        typedef std::shared_ptr<KeyFrameNode> Ptr;

        static BoWScoreT SIMILAR_VAL_THRES;   /// 关键帧相似度最低阈值

        /**
         * @brief 组合两个数据的构造函数
         * 
         * @param pKf 
         * @param pKfCloud 
         */
        KeyFrameNode(ORB_SLAM2::KeyFrame *pKf, PointCloudT::Ptr pKfCloud);

        /**
         * @brief 计算两个关键帧节点之间的相似度接口
         * 
         * @param rhs 比较关键帧节点
         * @return BoWScoreT 
         */
        BoWScoreT computeSimiliarity(const KeyFrameNode::Ptr &rhs); 

        /**
         * @brief 关键帧坐标系变换, 从原来本KeyFrameNode所属的地图坐标系(r)
         * 变换到目标地图坐标系(w): $T_{wf} = T_{wr}*T_{rf}$
         * 
         * @param Twr 
         * @return true
         * @return false
         */
        bool transformByPose(const cv::Mat &Twr);
        
        /**
         * @brief 返回使用关键帧位姿变换后的点云
         * 
         * @param dstCloud 
         * @return true 
         * @return false 
         */
        bool getTransformedCloud(PointCloudT::Ptr &dstCloud);

    protected:
        ORB_SLAM2::KeyFrame *mpKf;  /// 保存对应关键帧
        PointCloudT::Ptr mpKfCloud;  /// 保存对应关键帧点云
    };
} // namespace MultiMapMerge

#endif  // MULTIMAPMERGE_KEYFRAMENODE_H