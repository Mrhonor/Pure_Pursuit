#ifndef __PURE_PURSUIT__
#define __PURE_PURSUIT__


#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "track.h"
#include "math.h"
#include "arc_length_spline.h"
#include  <fstream>

using namespace std;




struct Input{
    double dD;
    double dDelta;
    double dVs;

    void setZero()
    {
        dD = 0.0;
        dDelta = 0.0;
        dVs = 0.0;
    }
};


class pure_pursuit
{
private:
    double lr;
    double lf;
    double last_s;
    double lookHeadDist;
    bool isSetTrack;
    Track cur_track;
    ArcLengthSpline track_;
    fstream outlog;

    // ros
    ros::Subscriber ekf_state_sub; // 状态向量订阅
    ros::Subscriber ref_path_sub;  // 参照路径订阅

    ros::Publisher control_pub;    // 控制向量输出

    
public:
    pure_pursuit(ros::NodeHandle &n);
    ~pure_pursuit();

    Input calcPurePursuit(const State& );

    State getRearState(const State&);
private:
    // 状态回调函数
    void ekfStateCallback(const std_msgs::Float64MultiArrayConstPtr& msg);

    // reference path 回调函数
    void refPathCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
};




#endif