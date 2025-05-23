#include "fuzzy_pid/fuzzyPID.h"


// 三角隶属函数
static float trimf(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0;
    return (x < b) ? (x - a)/(b - a) : (c - x)/(c - b);
}

// 高斯隶属函数
static float gaussmf(float x, float ave, float sigma) {
    return exp(-pow((x-ave)/sigma, 2));
}

// 梯形隶属函数 
static float trapmf(float x, float a, float b, float c, float d) {
    if (x < a || x > d) return 0;
    if (x >= b && x <= c) return 1;
    if (x >= a && x < b) return (x-a)/(b-a);
    return (d-x)/(d-c);
}

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
) {
    // 初始化PID参数
    pid->kp = kp_init;
    pid->ki = ki_init;
    pid->kd = kd_init;
    
    pid->integral = 0.0f;

    // 设置参数范围
    pid->kp_min = kp_min;
    pid->kp_max = kp_max;
    pid->ki_min = ki_min;
    pid->ki_max = ki_max;
    pid->kd_min = kd_min;
    pid->kd_max = kd_max;

    // 初始值限幅
    pid->kp = fmax(kp_min, fmin(kp_init, kp_max));
    pid->ki = fmax(ki_min, fmin(ki_init, ki_max));
    pid->kd = fmax(kd_min, fmin(kd_init, kd_max));

    // 设置模糊参数
    pid->error_max = error_max;
    pid->delta_max = delta_max;
    pid->delta_Kp_max = delta_Kp_max;
    pid->delta_Ki_max = delta_Ki_max;
    pid->delta_Kd_max = delta_Kd_max;
    
    // 复制模糊规则
    for(int i=0; i<qf_default; i++) {
        for(int j=0; j<qf_default; j++) {
            pid->Kp_rule_matrix[i][j] = Kp_rule[i][j];
            pid->Ki_rule_matrix[i][j] = Ki_rule[i][j];
            pid->Kd_rule_matrix[i][j] = Kd_rule[i][j];
        }
    }
    
    // 设置隶属函数参数
    strcpy(pid->mf_type_e, mf_type_e);
    strcpy(pid->mf_type_de, mf_type_de);
    
    // 分配内存并复制参数
    int e_param_size = (strcmp(mf_type_e, "trapmf") == 0) ? 4*qf_default : 
                     (strcmp(mf_type_e, "gaussmf") == 0) ? 2*qf_default : 3*qf_default;
    pid->e_mf_paras = static_cast<const float*>(malloc(e_param_size * sizeof(float)));
    memcpy(const_cast<float*>(pid->e_mf_paras), e_mf_params, e_param_size * sizeof(float));

    int de_param_size = (strcmp(mf_type_de, "trapmf") == 0) ? 4*qf_default : 
                      (strcmp(mf_type_de, "gaussmf") == 0) ? 2*qf_default : 3*qf_default;
    pid->de_mf_paras = static_cast<const float*>(malloc(de_param_size * sizeof(float)));
    memcpy(const_cast<float*>(pid->de_mf_paras), de_mf_params, de_param_size * sizeof(float));

    pid->d_filter_alpha = 0.2;  // 推荐值0.1-0.3
}

// 模糊推理核心
static float fuzzy_inference(struct FuzzyPID* pid, float e, float de, int rule_matrix[][qf_default]) {
    float u_e[qf_default], u_de[qf_default];
    
    // 误差模糊化
    for(int i=0; i<qf_default; i++) {
        if(strcmp(pid->mf_type_e, "trimf") == 0) {
            u_e[i] = trimf(e, pid->e_mf_paras[i*3], pid->e_mf_paras[i*3+1], pid->e_mf_paras[i*3+2]);
        } else if(strcmp(pid->mf_type_e, "gaussmf") == 0) {
            u_e[i] = gaussmf(e, pid->e_mf_paras[i*2], pid->e_mf_paras[i*2+1]);
        } else if(strcmp(pid->mf_type_e, "trapmf") == 0) {
            u_e[i] = trapmf(e, pid->e_mf_paras[i*4], pid->e_mf_paras[i*4+1], 
                           pid->e_mf_paras[i*4+2], pid->e_mf_paras[i*4+3]);
        }
    }
    
    // 误差变化率模糊化
    for(int i=0; i<qf_default; i++) {
        if(strcmp(pid->mf_type_de, "trimf") == 0) {
            u_de[i] = trimf(de, pid->de_mf_paras[i*3], pid->de_mf_paras[i*3+1], pid->de_mf_paras[i*3+2]);
        } else if(strcmp(pid->mf_type_de, "gaussmf") == 0) {
            u_de[i] = gaussmf(de, pid->de_mf_paras[i*2], pid->de_mf_paras[i*2+1]);
        } else if(strcmp(pid->mf_type_de, "trapmf") == 0) {
            u_de[i] = trapmf(de, pid->de_mf_paras[i*4], pid->de_mf_paras[i*4+1], 
                            pid->de_mf_paras[i*4+2], pid->de_mf_paras[i*4+3]);
        }
    }
    
    // 规则推理
    float numerator = 0, denominator = 0;
    for(int i=0; i<qf_default; i++) {
        for(int j=0; j<qf_default; j++) {
            float weight = fmin(u_e[i], u_de[j]);
            numerator += weight * rule_matrix[i][j];
            denominator += weight;
        }
    }
    return (denominator > 1e-6) ? numerator/denominator : 0;
}

// 参数自整定
void fuzzy_pid_auto_tune(struct FuzzyPID* pid, float error, float delta_error) {
    // 参数归一化
    float e_norm = error / pid->error_max * 3.0f;
    float de_norm = delta_error / pid->delta_max * 3.0f;
    
    // 模糊推理
    float delta_kp = fuzzy_inference(pid, e_norm, de_norm, pid->Kp_rule_matrix);
    float delta_ki = fuzzy_inference(pid, e_norm, de_norm, pid->Ki_rule_matrix);
    float delta_kd = fuzzy_inference(pid, e_norm, de_norm, pid->Kd_rule_matrix);
    
    // 参数更新
    pid->kp += delta_kp * pid->delta_Kp_max;
    pid->ki += delta_ki * pid->delta_Ki_max;
    pid->kd += delta_kd * pid->delta_Kd_max;
    
    // 限幅处理
    pid->kp = fmax(pid->kp_min, fmin(pid->kp, pid->kp_max));
    pid->ki = fmax(pid->ki_min, fmin(pid->ki, pid->ki_max));
    pid->kd = fmax(pid->kd_min, fmin(pid->kd, pid->kd_max));
}

// 控制量计算
float fuzzy_pid_control(struct FuzzyPID* pid, float error, float delta_error) {
    fuzzy_pid_auto_tune(pid, error, delta_error); // 参数自整定
    
    // 积分分离条件（误差绝对值大于80%时停止积分）
    if(fabs(error) > 0.8 * pid->error_max)
    {
        pid->integral += 0;  // 停止积分
    } 
    else
    {
        pid->integral += error;
    }
    
    // 积分限幅
    pid->integral = fmax(-pid->ki_max/pid->ki, fmin(pid->integral, pid->ki_max/pid->ki));
    
    // 应用一阶低通滤波
    pid->filtered_delta_error = pid->d_filter_alpha * delta_error + 
                               (1 - pid->d_filter_alpha) * pid->filtered_delta_error;
                               
    return pid->kp * error + pid->ki * pid->integral + pid->kd * pid->filtered_delta_error;
}