#include <glog/logging.h>

#include "MultiMapMerge/DiscreteGlobalMap.h"

namespace MultiMapMerge
{
    int DiscreteGlobalMap::SIMILAR_KF_THRES = 0.3;
    size_t DiscreteGlobalMap::nextId = 0;

    DiscreteGlobalMap::DiscreteGlobalMap(ORB_SLAM2::System *pSystem):
    mpSystem(pSystem) {
        mnId = nextId++;
        // TODO: ...
    }


    DiscreteGlobalMap::~DiscreteGlobalMap() {
        // TODO: ...
    }


    bool DiscreteGlobalMap::appnedNode(ORB_SLAM2::KeyFrame *pKf, PointCloudT::Ptr &pCloud) {
        // TODO: ...
    }


    /// 两地图拼接入口
    bool DiscreteGlobalMap::mergeGlobalMap(DiscreteGlobalMap::Ptr &rhs) {
        // 1. 找重叠区域关联的关键帧对
        std::unordered_map<size_t, size_t> mmapKfs = findOverlappingNodes(rhs);
        if (mmapKfs.size() < SIMILAR_KF_THRES) {
            LOG(WARNING) << "Fail to find ovelapping between map id=" << mnId << " and id=" << rhs->mnId 
            << ": Too few relative KeyFrame has been found (" << mmapKfs.size() << " / " << SIMILAR_KF_THRES << ")";
            return false;
        }
        // 2. 根据关键帧对点云配准计算
        cv::Mat Twr = cv::Mat::eye(4, 4, CV_32F);
        bool ret = computeTransformPose(Twr, rhs, mmapKfs);
        if (!ret) {
            LOG(WARNING) << "Fail to compute transform between map id=" << mnId << " and id=" << rhs->mnId;
            return false;
        }
        // 3. 合并rhs到调用地图中
        for (auto pKfNode : rhs->mvpKfNodes) {
            pKfNode->transformByPose(Twr);
            mvpKfNodes.push_back(pKfNode);
        }
        return true;
    }


    /// 返回点云拼接后地图
    PointCloudT::Ptr DiscreteGlobalMap::getGlobalPointCloud() const {
        // TODO: ...
    }


    /// 搜索重叠区域的关联关键帧
    std::unordered_map<size_t, size_t> 
    DiscreteGlobalMap::findOverlappingNodes(DiscreteGlobalMap::Ptr &rhs) {
        // TODO: ...
    }


    /// 根据关联关键帧计算Twr入口
    bool 
    DiscreteGlobalMap::computeTransformPose(
        cv::Mat &dst, 
        DiscreteGlobalMap::Ptr &rhs, 
        const std::unordered_map<size_t, size_t> &umapKfs) {

        // TODO: ...
    }


    /// 点云配准入口
    bool
    DiscreteGlobalMap::matchPointCloud(
        cv::Mat &dst, 
        PointCloudT::Ptr &wCloud, const cv::Mat &Two, 
        PointCloudT::Ptr &rCloud, const cv::Mat &Tro) {
        
        // TODO: ...
    }

} // namespace MultiMapMerge
