#include "MultiMapMerge/MapMerger.h"

namespace MultiMapMerge
{
    const char *MapMerger::MAP_PATHNAME = "mappoints.bin";
    const char *MapMerger::PCD_DIRNAME = "kf_clouds";
    MapMerger::Ptr MapMerger::mpMerger = nullptr;

    MapMerger::Ptr MapMerger::getInstance(const std::vector<std::string> &vstrMapParentDirs) {
        if (mpMerger == nullptr) {
            mpMerger = MapMerger::Ptr(new MapMerger(vstrMapParentDirs));
        }
        return mpMerger;
    }


    bool MapMerger::merge() {
        // TODO: ...
    }


    PointCloudT MapMerger::getMergedCloud() {
        // TODO: ...
    }


    MapMerger::MapMerger(const std::vector<std::string> &vstrMapParentDirs) {
        // TODO...
    }
} // namespace MultiMapMerge
