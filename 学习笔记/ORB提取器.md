# 函数

ComputeKeyPointsOctTree

## 流程

1. 遍历每一层图像
    1. 图像分成（nCols ,nRows）个网格cell，每个网格所占的行数和列数是（wCell,hCell），wCell,hCell约等于W，W是35  
       ~~ 感觉写复杂了，直接用W不就行了？~~ 这样覆盖范围更大一些
    2. 开始遍历图像网格，还是以行开始遍历的
        1. 算一些坐标，包括每个网格的位置
        2. 开始列的遍历
            1. 算一些坐标
            2. 提FAST，如果没能检测到特征点就阈值更低的FAST再提一次，
               这里用来提FAST的范围是网格上下左右各扩充`EDGE_THRESHOLD=3`，结果在vKeysCell
            3. 恢复特征点其在图像坐标系下的尺度，存在`vToDistributeKeys`中