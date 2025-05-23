#include "fuzzy_pid/fuzzy_control.h"

// 模糊规则定义
// 修改Kp规则矩阵（增强正误差区的比例作用）
const int FuzzyControl::Kp_rule[qf_default][qf_default] = {
    {PB, PB, PB, PM, PS, ZO, ZO},     // 原第0行增加PB
    {PB, PB, PM, PS, PS, ZO, NS},     // 保持
    {PM, PM, PM, PS, ZO, NS, NS},     // 保持
    {PM, PM, PS, ZO, NS, NM, NM},     // 保持
    {PS, PS, ZO, NS, NM, NM, NB},     // 原第4行NS->NM 
    {ZO, ZO, NS, NM, NM, NB, NB},     // 原第5行PS->ZO
    {ZO, ZO, NM, NM, NB, NB, NB}      // 原第6行增强负向
};

const int FuzzyControl::Ki_rule[qf_default][qf_default] = {
    {NB, NB, NM, NM, NS, ZO, ZO},
    {NB, NB, NM, NS, NS, ZO, ZO},
    {NM, NM, NS, NS, ZO, PS, PS},
    {NM, NS, NS, ZO, PS, PM, PM},
    {NS, NS, ZO, PS, PS, PM, PB},
    {ZO, ZO, PS, PS, PM, PB, PB},
    {ZO, ZO, PS, PM, PM, PB, PB}
};


// 修改Kd规则矩阵（增强动态抑制能力）
const int FuzzyControl::Kd_rule[qf_default][qf_default] = {
    {PB, NS, NB, NB, NB, NM, PB},    // 首尾增强
    {PS, NS, NM, NM, NM, NS, PS},     // 增强边界
    {ZO, NS, NS, NM, NS, NS, ZO},    
    {ZO, ZO, PB, PB, PB, ZO, ZO},    
    {PB, PB, PB, PM, PM, ZO, ZO},    // 扩大正区作用
    {ZO, PS, PM, PM, PM, PS, PB},    // 增强右半区
    {PB, PS, PM, PB, PB, PM, PB}     // 全正区增强
};

// 隶属函数参数（三角隶属函数）
const float FuzzyControl::e_mf_params[] = {
    -3,-3,-2,   // NB
    -3,-2,-1,   // NM
    -2,-1,0,    // NS
    -1,0,1,     // ZO
    0,1,2,      // PS
    1,2,3,      // PM
    2,3,3       // PB
};

const float FuzzyControl::de_mf_params[] = {
    -3,-3,-2,   // NB
    -3,-2,-1,   // NM
    -2,-1,0,    // NS
    -1,0,1,     // ZO
    0,1,2,      // PS
    1,2,3,      // PM
    2,3,3       // PB
};

// ==== 构造函数 ====
FuzzyControl::FuzzyControl()
{
    // 初始化四个自由度的控制器
    for(int i=0; i<DOF; ++i)
    {
        if (i == 0)
        {
            fuzzy_pid_init(
                &pid_controllers[i],
                3.2f,    // 大幅提升初始Kp：2.1->3.2（增强比例作用）
                0.06f,   // 降低积分项：0.1->0.06（防止过冲）
                0.85f,    // 增强微分项：0.45->0.85（抑制震荡）
                0.8f, 6.0f,   // 扩大Kp范围：[0.5,5.0]->[0.8,6.0]
                0.005f, 0.15f,// 缩小Ki范围：[0.01,0.2]->[0.005,0.15]
                0.3f, 2.5f,  // 扩大Kd范围：[0.2,1.5]->[0.3,2.5]
                4.0f,     // 缩小误差上限：5.0->4.0（加速进入稳定区）
                1.2f,     // 放宽误差变化率：1.0->1.2（允许更快调整）
                0.2f,     // 增加Kp调整幅度：0.15->0.2（加快P响应）
                0.02f,    // 缩小Ki调整幅度：0.03->0.02
                0.4f,     // 增加Kd调整幅度：0.25->0.4（增强D调节）
                (int(*)[qf_default])Kp_rule,
                (int(*)[qf_default])Ki_rule,
                (int(*)[qf_default])Kd_rule,
                "trimf", e_mf_params,
                "trimf", de_mf_params
            );
        }
        else if (i == 1)
        {
            fuzzy_pid_init(
                &pid_controllers[i],
                2.1f,   // kp从2.0->2.1（增加比例作用，提高响应速度）
                0.1f,   // ki从0.08->0.1（进一步降低静差）
                0.45f,   // kd从0.4->0.45（增强微分抑制震荡）
                0.5f, 5.0f,  // kp范围[0.5,5.0]
                0.01f, 0.2f, // ki范围[0.01,0.3] --> [0.01,0.2]
                0.2f, 1.5f,  // kd范围[0.1,1.0] --> [0.2,1.5]
                5.0f,   // error保持5.0
                1.0f,   // delta_max从0.5->1.0（适应更大变化率）
                0.15f,   // delta_Kp_max从0.3->0.2（限制KP调整幅度）
                0.03f,  // delta_Ki_max保持0.05
                0.25f,   // delta_Kd_max从0.1->0.2（允许更大KD调整）
                (int(*)[qf_default])Kp_rule,
                (int(*)[qf_default])Ki_rule,
                (int(*)[qf_default])Kd_rule,
                "trimf", e_mf_params,
                "trimf", de_mf_params
            );
        }
        else if (i == 2)
        {
            fuzzy_pid_init(
                &pid_controllers[i],
                2.0f,    // 降低Kp：3.5->2.2->2.0（减少推力突变）
                0.03f,   // 减小Ki：0.05->0.03（降低积分效应）
                1.8f,    // 提升Kd：1.2->1.8（增强镇定）
                0.8f, 4.0f,   // 缩小Kp范围：[1.5,8.0]->[0.8,4.0]
                0.002f, 0.08f,// 缩小Ki范围：[0.005,0.1]->[0.002,0.08]
                0.8f, 3.0f,  // 调整Kd范围：[0.5,4.0]->[0.8,3.0]
                5.0f,    // 误差上限恢复为5.0
                1.0f,    // 降低delta_max：1.5->1.0
                0.15f,   // 减小Kp调整幅度：0.3->0.15
                0.01f,  // 严格限制Ki调整：0.02->0.01
                0.3f,    // 减小Kd调整幅度：0.4->0.3
                (int(*)[qf_default])Kp_rule,
                (int(*)[qf_default])Ki_rule,
                (int(*)[qf_default])Kd_rule,
                "trimf", e_mf_params,
                "trimf", de_mf_params
            );
        }
        else
        {
            fuzzy_pid_init(
                &pid_controllers[i],
                1.4f,    // 精细调整Kp：1.6->1.4
                0.006f,  // 降低Ki：0.012->0.006
                0.45f,   // 优化Kd：0.38->0.45
                0.6f, 2.0f,  // 缩小Kp范围：[0.8,2.2]->[0.6,2.0]
                0.0008f, 0.025f,// 收紧Ki范围
                0.5f, 1.8f,  // 调整Kd范围
                2.0f,    // 恢复误差上限2.0
                0.35f,   // 降低变化率限
                0.09f,   // 减小Kp调整幅度
                0.002f,  // 严格Ki调整
                0.15f,   // 降低Kd调整
                (int(*)[qf_default])Kp_rule,
                (int(*)[qf_default])Ki_rule,
                (int(*)[qf_default])Kd_rule,
                "trimf", e_mf_params,
                "trimf", de_mf_params
            );
        }
    }

    // 订阅话题
    moment_sub = nh_.subscribe("/xtdrone/iris_0/image_moment", 10, &FuzzyControl::momentCallback, this);
    
    // 话题发布
    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/xtdrone/iris_0/cmd_vel_flu", 10);

    for(int i = 0; i < DOF; ++i)
    {
        std::string topic_name = "/pid_params/dof" + std::to_string(i);
        pid_params_pubs[i] = nh_.advertise<fuzzy_pid::PIDParams>(topic_name, 10);
    }
}

// ==== 回调函数 ====
void FuzzyControl::momentCallback(const image_moment::Moment::ConstPtr& msg)
{
    geometry_msgs::Twist cmd_vel;
    float errors[DOF] = 
    {
        static_cast<float>(msg->qx - TARGET_QX),    // X轴误差
        static_cast<float>(msg->qy - TARGET_QY),    // Y轴误差
        static_cast<float>(msg->qh - TARGET_QH),    // 高度误差
        static_cast<float>(msg->qpsi - TARGET_QPSI) // 偏航角误差
    };

    // 动态计算最大误差（示例）
    static float max_observed_error = 0;
    max_observed_error = fmax(max_observed_error, fabs(errors[0]));
    pid_controllers[0].error_max = max_observed_error * 1.2;  // 保持20%裕量
    
    for (int i = 0; i < DOF; ++i) 
    {
        float delta_error = errors[i] - prev_errors[i];
        prev_errors[i] = errors[i];  // 更新历史误差

        // 计算模糊PID输出
        float output = fuzzy_pid_control(&pid_controllers[i], errors[i], delta_error);
        
        // 限幅输出
        switch(i)
        {
            case 0: 
            {
                output = fmaxf(fminf(output, 2.0f), -2.0f); 
                cmd_vel.linear.y = -output;
                break;
            }
            case 1: 
            {
                output = fmaxf(fminf(output, 2.0f), -2.0f); 
                cmd_vel.linear.z = -output;
                break;
            }
            case 2: 
            {
                output = fmaxf(fminf(output, 2.0f), -2.0f); 
                cmd_vel.linear.x = output;
                break;
            }
            case 3: 
            {
                // 零区死带处理
                const float dead_zone = 0.15f; // ±0.15度不响应
                if(fabs(errors[3]) < dead_zone) {
                    output = 0;
                    pid_controllers[i].integral *= 0.5f; // 半衰积分
                }
                
                // 动态微分滤波（误差越小滤波越强）
                static float prev_deriv = 0;
                float alpha = fmaxf(0.3f, 1.0f - fabs(errors[3])/2.0f); // 零区α=0.85
                float filtered_deriv = alpha * prev_deriv + (1-alpha) * delta_error;
                prev_deriv = filtered_deriv;
                output = pid_controllers[i].kp * errors[3] 
                    + pid_controllers[i].ki * pid_controllers[i].integral
                    + pid_controllers[i].kd * filtered_deriv; // 使用滤波后的微分
                
                // 非线性增益调整（误差越小增益越低）
                float gain_factor = fminf(1.0f, 0.5f + fabs(errors[3])/1.0f);
                output *= gain_factor;
                
                // 改进型低通滤波（双重滤波）
                static float filtered_output[2] = {0, 0};
                filtered_output[0] = 0.6f * filtered_output[0] + 0.4f * output;
                filtered_output[1] = 0.7f * filtered_output[1] + 0.3f * filtered_output[0];
                
                // 动态速率限制（根据误差大小调整）
                static float last_output = 0;
                float rate_limit = (fabs(errors[3]) > 1.0f) ? 0.25f : 0.12f;
                output = fmaxf(fminf(filtered_output[1], last_output + rate_limit), 
                              last_output - rate_limit);
                last_output = output;
                
                // 非线性输出映射（小误差区降低增益）
                if(fabs(errors[3]) < 0.5f) 
                {
                    output *= 0.7f;
                }
                
                cmd_vel.angular.x = -fmaxf(fminf(output, 0.9f), -0.9f);
                break;
            }   
        }
    }

    // 发布控制指令
    cmd_vel_pub.publish(cmd_vel);
    ROS_INFO("Control CMD: x=%.2f y=%.2f z=%.2f roll=%.2f", 
        cmd_vel.linear.x, cmd_vel.linear.y, 
        cmd_vel.linear.z, cmd_vel.angular.x
    );
    
    for(int pid_index = 0; pid_index < DOF; ++pid_index) 
    { // 只监控前两个控制器
        fuzzy_pid::PIDParams params_msg;
        params_msg.index = pid_index;
        params_msg.kp = pid_controllers[pid_index].kp;
        params_msg.ki = pid_controllers[pid_index].ki;
        params_msg.kd = pid_controllers[pid_index].kd;
        pid_params_pubs[pid_index].publish(params_msg);
    }
}