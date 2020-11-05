#ifndef MULTIMAPMERGE_COMMON_H
#define MULTIMAPMERGE_COMMON_H

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace MultiMapMerge {
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef double BoWScoreT;

    // 前向声明
    class DiscreteGlobalMap;
    struct KeyFrameNode;
}

#endif  // MULTIMAPMERGE_COMMON_H