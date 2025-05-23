#ifndef FUZZY_CONTROL_H
#define FUZZY_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "image_moment/Moment.h"
#include "fuzzy_pid/PIDParams.h"
#include "fuzzy_pid/fuzzyPID.h"

#define DOF 4 

class FuzzyControl
{
private:
    ros::NodeHandle nh_;
    
    // 订阅/发布器
    ros::Subscriber moment_sub;
    ros::Publisher cmd_vel_pub;
    
    ros::Publisher pid_params_pubs[DOF];

    // 四个自由度的模糊PID控制器
    FuzzyPID pid_controllers[DOF];
    float prev_errors[DOF] = {0};

    // 目标值
    const float TARGET_QX = 1.0f;
    const float TARGET_QY = 0.0f;
    const float TARGET_QH = 0.0f;
    const float TARGET_QPSI = 0.0f;
    
    // 模糊规则定义
    static const int Kp_rule[qf_default][qf_default];
    static const int Ki_rule[qf_default][qf_default];
    static const int Kd_rule[qf_default][qf_default];
    
    // 隶属函数参数
    static const float e_mf_params[];  // 三角隶属函数7组*3参数
    static const float de_mf_params[];

    // PID参数
    float kp = 2.0f;    // 比例参数
    float ki = 0.1f;    // 积分参数
    float kd = 0.05f;   // 微分参数
    float error_max = 10.0f;  // 最大误差范围
    float delta_max = 0.5f;   // 最大误差变化率范围
    

public:
    FuzzyControl();
    ~FuzzyControl(){};
    
    void momentCallback(const image_moment::Moment::ConstPtr& msg);
};

#endif

