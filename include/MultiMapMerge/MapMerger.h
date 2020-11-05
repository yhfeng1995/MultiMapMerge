/**
 * @file MapMerger.h
 * @author Yonghui Feng (yhfeng1995@gmail.com)
 * @brief 多地图拼接的调用类
 * @version 0.1
 * @date 2020-11-05
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef MULTIMAPMERGE_MAPMERGER_H
#define MULTIMAPMERGE_MAPMERGER_H

#include "MultiMapMerge/common.h"
#include "MultiMapMerge/DiscreteGlobalMap.h"

namespace MultiMapMerge
{
    class MapMerger {
    public:
        typedef std::shared_ptr<MapMerger> Ptr;
        static const char *MAP_PATHNAME;  /// ${MAP_PARENT_DIR}/MAP_PATHNAME
        static const char *PCD_DIRNAME;  /// ${MAP_PARENT_DIR}/PCD_DIRNAME/

        /**
         * @brief 单例实例获取接口
         * 
         * @param vstrMapParentDirs 
         * @return MapMerger::Ptr 
         */
        MapMerger::Ptr getInstance(const std::vector<std::string> &vstrMapParentDirs = std::vector<std::string>());

        /**
         * @brief 多机器人地图拼接入口函数
         * 
         * @return true 
         * @return false 
         */
        bool merge();

        /**
         * @brief 返回拼接后点云地图
         * 
         * @return PointCloudT 
         */
        PointCloudT getMergedCloud();

    private:
        MapMerger(const std::vector<std::string> &vstrMapParentDirs);

        MapMerger(const MapMerger &mereger) {}

        MapMerger& operator=(const MapMerger &merger) {}

        static MapMerger::Ptr mpMerger;  /// MapMerger单例实例
        std::vector<DiscreteGlobalMap::Ptr> mvpGlobalMaps;  /// 多机器人保存的独立全局地图列表
        DiscreteGlobalMap::Ptr mpTargetMap;  /// 拼接成功后返回的参考地图
        
    };
} // namespace MultiMapMerge


#endif  // MULTIMAPMERGE_MAPMERGER_H