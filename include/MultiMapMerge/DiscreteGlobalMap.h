/**
 * @file DiscreteGlobalMap.h
 * @author Yonghui Feng (yhfeng1995@gmail.com)
 * @brief 单机器人构建地图数据结构定义
 * @version 0.1
 * @date 2020-11-05
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef MULTIMAPMERGE_DISCRETEGLOBALMAP_H
#define MULTIMAPMERGE_DISCRETEGLOBALMAP_H

#include "MultiMapMerge/KeyFrameNode.h"

namespace ORB_SLAM2 {
    class System;
    class KeyFrame;
}

namespace MultiMapMerge
{
    /**
     * @brief 单机器人构建的全局地图离散集合, 保存了SLAM
     * 的所有关键帧节点(KeyFrameNode)集合, 提供两个独立
     * 全局地图之间的重叠场景关键帧搜索与位姿校正
     */
    class DiscreteGlobalMap {
    public:
        typedef std::shared_ptr<DiscreteGlobalMap> Ptr;

        static int SIMILAR_KF_THRES;  /// 地图重叠的相似关键帧数最低阈值

        /**
         * @brief 传入对应ORB-SLAM2系统, 初始化一个空地图
         * 
         * @param pSystem 
         */
        DiscreteGlobalMap(ORB_SLAM2::System *pSystem);

        /**
         * @brief 单机器人地图类负责关联ORB-SLAM2实例的析构
         * 
         */
        ~DiscreteGlobalMap();

        /**
         * @brief 向地图中添加一个关键帧节点
         * 
         * @param pKf 
         * @param pCloud 
         * @return true 
         * @return false 
         */
        bool appnedNode(ORB_SLAM2::KeyFrame *pKf, PointCloudT::Ptr &pCloud);

        /**
         * @brief 合并两个独立的全局地图, 如果能够合并
         * 传入地图的所有关键帧节点会添加到调用地图结构中
         * 
         * @param rhs 
         * @return true 
         * @return false 
         */
        bool mergeGlobalMap(DiscreteGlobalMap::Ptr &rhs);

        /**
         * @brief 获得拼接后完整地图
         * 
         * @return PointCloudT::Ptr 
         */
        PointCloudT::Ptr getGlobalPointCloud() const ;
                
        size_t getId() const { return mnId; }

    protected:
        /**
         * @brief 根据关键帧之间的相似度计算, 提取出两个地图中关键帧的相似关键帧匹配对
         * 
         * @param rhs 
         * @return std::unordered_map<size_t, size_t> 关联数不大于SIMILAR_KF_THRES地图重叠查找失败
         */
        std::unordered_map<size_t, size_t> findOverlappingNodes(DiscreteGlobalMap::Ptr &rhs);

        /**
         * @brief 根据重叠关键帧匹配对, 使用点云配准估计两个地图原点关键帧之间的$T_{wr}$
         * 传入地图的每个关键帧位姿$T_{rf}$变换为: $T_{wf} = T{wr}*T_{rf}$
         * 
         * @param dst 返回空Pose可以表示失败
         * @param rhs 
         * @param umapKfs 
         * @return true
         * @return false
         */
        bool computeTransformPose(cv::Mat &dst, DiscreteGlobalMap::Ptr &rhs, const std::unordered_map<size_t, size_t> &umapKfs);

        /**
         * @brief 两片稠密点云配准的具体实现
         * 
         * @param dst 返回空Pose可以表示失败
         * @param wCloud 
         * @param Two 
         * @param rCloud 
         * @param Tro 
         * @return true
         * @return false
         */
        bool matchPointCloud(cv::Mat &dst, PointCloudT::Ptr &wCloud, const cv::Mat &Two, 
                             PointCloudT::Ptr &rCloud, const cv::Mat &Tro);

        static size_t nextId;  /// MapMerger全局ID
        size_t mnId;  /// 当前实例ID
        ORB_SLAM2::System *mpSystem;  /// ORB_SLAM2库
        std::vector<KeyFrameNode::Ptr> mvpKfNodes;  // 关键帧节点集合
    };

} // namespace MultiMapMerge


#endif  // MULTIMAPMERGE_DISCRETEGLOBALMAP_H