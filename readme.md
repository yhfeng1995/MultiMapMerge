# 多机器人地图拼接

## UML图

![](http://assets.processon.com/chart_image/5fa39a6e1e08534d55689e5b.png)

## TODO LIST

- `KeyFrameNode`
    
    - [ ] 计算相似度(`computeSimilarity`)
    - [ ] 关键帧地图坐标系转移(`transformByPose`)

- `DiscreteGlobalMap`

    - [ ] 搜索两个地图间重叠区域匹配关键帧对(`findOverlappingNodes`)
    - [ ] 根据关键帧匹配对计算地图变换位姿(`computeTransformPose`)
    - [ ] 点云配准具体实现(`matchPointcloud`)

- `MapMerger`

    - [ ] 地图拼接主入口(`merge`)
    - [ ] 数据输入(`MapMerger`构造)

## CHANGELOG

- 2020/11/05 22:25
    - 提交程序框架和大纲