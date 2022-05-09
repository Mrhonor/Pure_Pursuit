# 纯跟踪算法节点
## 节点说明
1. 该节点基于纯跟随算法实现小车的控制，建议搭配[核心节点](https://github.com/Mrhonor/smart_car_mc110)使用
2. 该节点首先接收离散点目标轨迹 **/RefPath**，进行轨迹弧长参数化。根据状态向量 **/EKF/State**求轨迹上的投影点，在投影点上加一个预瞄距离的弧长作为预瞄点，求解控制向量
3. 通过 **/MPCC/Control**发布控制向量
4. 需要先将[Eigen库](https://gitlab.com/libeigen/eigen)克隆下来。新建External文件夹，将Eigen库移至External文件夹下。
5. 运行方式
```
rosrun pure_pursuit pure_pursuit
```