# 变量

+ mfNNratio  最小距离 < mfNNratio*次小距离，构造函数第一个参数
+ mbCheckOrientation  是否检查旋转

# 函数
## 构造函数
需要的匹配距离和是否检查旋转

# SearchByProjection MapPoint
Tracking::SearchLocalPoints用，SearchLocalPoints被TrackLocalMap()调用

# SearchByProjection Frame
TrackWithMotionModel()调用
```c++
int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
```

# SearchByBoW Frame
1. Track::TrackReferenceKeyFrame()
2. Tracking::Relocalization() {
```c++
int ORBmatcher::SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
```

# SearchByBoW KeyFrame
回环用
```c++
int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
```

# SearchByProjection sim3
回环用
# SearchByProjection sim3 MapPoint KeyFrame
回环用

