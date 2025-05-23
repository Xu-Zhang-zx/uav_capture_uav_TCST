#ifndef FUZZY_PID_H
#define FUZZY_PID_H

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define qf_default 7

// 模糊语言值宏
#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

// 模糊PID结构体（支持参数自整定）
struct FuzzyPID {
    // PID动态参数
    float kp, ki, kd;

    float integral;         // 积分项
    
    // 模糊控制参数
    float error_max;        // 误差最大值
    float delta_max;        // 误差变化率最大值
    float delta_Kp_max;     // Kp调整幅度限制
    float delta_Ki_max;     // Ki调整幅度限制
    float delta_Kd_max;     // Kd调整幅度限制
    
    // 模糊规则库
    int Kp_rule_matrix[qf_default][qf_default];
    int Ki_rule_matrix[qf_default][qf_default];
    int Kd_rule_matrix[qf_default][qf_default];
    
    // 隶属函数参数
    const float *e_mf_paras;      // 误差隶属函数参数
    const float *de_mf_paras;     // 误差变化率隶属函数参数
    char mf_type_e[10];     // 误差隶属函数类型
    char mf_type_de[10];    // 误差变化率隶属函数类型
    
    // 中间变量
    float e_pre_1;          // 上一次误差
    float e_pre_2;          // 上上次误差

    float kp_min, kp_max;
    float ki_min, ki_max;
    float kd_min, kd_max;

    float d_filter_alpha;  // 微分滤波系数（0.1-0.3）
    float filtered_delta_error;
};

// 初始化函数
void fuzzy_pid_init(
    struct FuzzyPID* pid,
    float kp_init, float ki_init, float kd_init,
    float kp_min, float kp_max,
    float ki_min, float ki_max,
    float kd_min, float kd_max,
    float error_max, float delta_max,
    float delta_Kp_max, float delta_Ki_max, float delta_Kd_max,
    int Kp_rule[][qf_default], int Ki_rule[][qf_default], int Kd_rule[][qf_default],
    const char* mf_type_e, const float* e_mf_params,
    const char* mf_type_de, const float* de_mf_params
);

// 参数自整定函数
void fuzzy_pid_auto_tune(struct FuzzyPID* pid, float error, float delta_error);

// 控制量计算函数
float fuzzy_pid_control(struct FuzzyPID* pid, float error, float delta_error);

#endif