#include "high_speed_interception/control_only.h"

template<typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

// 斜对称矩阵函数
Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),
        v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;
    return m;
}

HighSpeedInterceptionControl::HighSpeedInterceptionControl() : nh_("~")
{
    // 初始化参数
    initParams();

    // 订阅话题
    iris_imu_sub_ = nh_.subscribe("/iris_0/mavros/imu/data", 200, 
                            &HighSpeedInterceptionControl::imuCallback_iris, this);
    typhoon_imu_sub_ = nh_.subscribe("/typhoon_h480_0/mavros/imu/data", 10, 
                            &HighSpeedInterceptionControl::imuCallback_typhoon, this);
    
    iris_pos_sub_ = nh_.subscribe("/iris_0/mavros/local_position/pose", 10,
                            &HighSpeedInterceptionControl::posCallback_iris, this);
    typhoon_pos_sub_ = nh_.subscribe("/typhoon_h480_0/mavros/local_position/pose", 10,
                            &HighSpeedInterceptionControl::posCallback_typhoon, this);

    iris_vel_sub_ = nh_.subscribe("/iris_0/mavros/local_position/velocity_local", 10,
                            &HighSpeedInterceptionControl::velCallback_iris, this);
    typhoon_vel_sub_ = nh_.subscribe("/typhoon_h480_0/mavros/local_position/velocity_local", 10,
                            &HighSpeedInterceptionControl::velCallback_typhoon, this);  

    // 发布控制指令
    cmd_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/iris_0/mavros/setpoint_raw/attitude",10);

}

// 控制器参数初始化
void HighSpeedInterceptionControl::initParams()
{
    // 控制器参数
    nh_.param("k1", k1_, 0.8);
    nh_.param("k2", k2_, 0.8);
    nh_.param("k_b", k_b_, 20.0);
    nh_.param("mass", mass_, 1.55);
    nh_.param("max_thrust", max_thrust_, 24.0);
    // nh_.param("max_thrust", max_thrust_, 28.26);
    // nh_.param("max_thrust", max_thrust_, 10.0);
    nh_.param("max_ang_vel", max_ang_vel_, 0.3);

    ROS_INFO("HighSpeedInterceptionControl have been initialized");
}

// IMU回调
void HighSpeedInterceptionControl::imuCallback_iris(const sensor_msgs::Imu::ConstPtr& msg)
{
    q_iris = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, 
                                msg->orientation.y, msg->orientation.z);
}

void HighSpeedInterceptionControl::imuCallback_typhoon(const sensor_msgs::Imu::ConstPtr& msg)
{
    q_typhoon = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                                   msg->orientation.y, msg->orientation.z);
}

// 位置回调
void HighSpeedInterceptionControl::posCallback_iris(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    iris_pos.pose = msg->pose;
}

void HighSpeedInterceptionControl::posCallback_typhoon(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    typhoon_pos.pose = msg->pose;
}

// 速度回调
void HighSpeedInterceptionControl::velCallback_iris(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    iris_vel.x = msg->twist.linear.x;
    iris_vel.y = msg->twist.linear.y;
    iris_vel.z = msg->twist.linear.z;
}

void HighSpeedInterceptionControl::velCallback_typhoon(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    typhoon_vel.x = msg->twist.linear.x;
    typhoon_vel.y = msg->twist.linear.y;
    typhoon_vel.z = msg->twist.linear.z;
}

// 核心控制算法
void HighSpeedInterceptionControl::updateControl()
{

    // ROS_INFO("iris_pos = [%f, %f, %f], typhoon_pos = [%f, %f, %f]",
    //     iris_pos.pose.position.x - 3, iris_pos.pose.position.y, iris_pos.pose.position.z,
    //     typhoon_pos.pose.position.x + 5, typhoon_pos.pose.position.y, typhoon_pos.pose.position.z);


    // 将机体速度转换到世界坐标系
    Eigen::Vector3d iris_vel_body(iris_vel.x, iris_vel.y, iris_vel.z);
    Eigen::Vector3d typhoon_vel_body(typhoon_vel.x, typhoon_vel.y, typhoon_vel.z);
    
    // 使用四元数进行坐标系转换
    Eigen::Vector3d iris_vel_world = q_iris.toRotationMatrix() * iris_vel_body;
    Eigen::Vector3d typhoon_vel_world = q_typhoon.toRotationMatrix() * typhoon_vel_body;

    // 计算相对位置和速度
    Eigen::Vector3d rel_pos(
        iris_pos.pose.position.x - typhoon_pos.pose.position.x - 8,
        iris_pos.pose.position.y - typhoon_pos.pose.position.y,
        iris_pos.pose.position.z - typhoon_pos.pose.position.z
    );

    Eigen::Vector3d rel_vel = iris_vel_world - typhoon_vel_world;  // 使用世界坐标系速度

    
    // 安全距离检查（防止除以零）
    double rel_dist = rel_pos.norm();
    if (rel_dist < 0.001)
    {
        ROS_WARN_THROTTLE(1.0, "Target is too close (%.2f m), control not executed", rel_dist);
        return;
    }

    // ------ 控制律计算 ------
    
    // 动态更新n_td_

    // 计算当前光轴在世界坐标系下的方向
    Eigen::Matrix3d R_eb = q_iris.toRotationMatrix();
    Eigen::Vector3d n_td_world = R_eb * camera_axis_body_;
    n_td_ = n_td_world.normalized();

    // 论文公式(4)：目标单位向量计算
    Eigen::Vector3d n_t = -rel_pos.normalized(); // EFCS中的LOS向量
    
    // 论文公式(5)：视线跟踪误差
    double z1 = 1.0 - n_td_.dot(n_t);
    
    // 论文公式(12)：Barrier Lyapunov安全约束
    // 安全裕度
    const double safe_margin = 0.05; 
    double k_b_safe = k_b_ - safe_margin;
    // 修正安全平方
    double k_b_sq_safe = k_b_safe * k_b_safe;
    
    // 记录上一次的z1值，用于平滑处理
    static double z1_prev = 0.0;
    static bool first_run = true;
    
    // 第一次运行时初始化
    if (first_run)
    {
        z1_prev = z1;
        first_run = false;
    }
    
    // 检查z1是否接近临界值
    if (z1 * z1 >= k_b_sq_safe)
    { 
        // 修正判断条件
        // 使用平滑过渡而非硬限制
        double transition_alpha = 0.3;
        z1 = (1-transition_alpha) * z1_prev + transition_alpha * sqrt(k_b_sq_safe);
        ROS_WARN_THROTTLE(1.0, "Line-of-sight error is close to the critical value (z1=%.4f, limit=%.4f)", 
                        z1, sqrt(k_b_sq_safe));
    }
    
    // 存储当前z1值用于下次迭代
    z1_prev = z1;
    
    // 使用更稳定的计算方式计算Barrier Lyapunov函数及其导数
    double barrier_term;
    double remaining_margin = k_b_sq_safe - z1*z1;
    if (remaining_margin < 1e-6) 
    { 
        // 防止除零
        remaining_margin = 1e-6;
    }
    
    barrier_term = z1 / remaining_margin;

    // 限制barrier_term最大值
    // const double max_barrier_term = 10.0;
    // barrier_term = std::min(barrier_term, max_barrier_term);
    
    // 论文公式(13)：角速度控制项
    Eigen::Vector3d b_omega1 = barrier_term * R_eb.transpose() * n_td_.cross(n_t);

    // 打印R_eb和R_eb.transpose()矩阵
    // Eigen::Matrix3d R_be = R_eb.transpose(); // 从世界坐标系到机体坐标系的转换
    // ROS_INFO_THROTTLE(2.0, "R_eb (body to world):\n%f %f %f\n%f %f %f\n%f %f %f", 
    //          R_eb(0,0), R_eb(0,1), R_eb(0,2),
    //          R_eb(1,0), R_eb(1,1), R_eb(1,2),
    //          R_eb(2,0), R_eb(2,1), R_eb(2,2));
             
    // ROS_INFO_THROTTLE(2.0, "R_be = R_eb.transpose() (world to body):\n%f %f %f\n%f %f %f\n%f %f %f", 
    //          R_be(0,0), R_be(0,1), R_be(0,2),
    //          R_be(1,0), R_be(1,1), R_be(1,2),
    //          R_be(2,0), R_be(2,1), R_be(2,2));

    // 论文公式(16)：期望相对速度
    Eigen::Vector3d v_rd = -k1_ * rel_pos;
    Eigen::Vector3d z2 = rel_vel - v_rd;

    // 论文公式(19)：期望加速度计算
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    // 不同于论文，未乘以质量m
    Eigen::Vector3d a_d = -k1_*rel_vel - k2_*z2 - rel_pos + 
                        barrier_term * (1/rel_dist) *
                        (-I + n_t*n_t.transpose())*n_td_;

    // ------ 推力计算（论文公式23）------
    // 阻尼系数 * 当前速度
    Eigen::Vector3d damping = -0.2 * iris_vel_world;
    // 加入阻尼项
    a_d = a_d + damping;
    // 考虑重力补偿（EFCS中重力向量）
    Eigen::Vector3d gravity(0, 0, -9.81); // 根据EFCS定义调整符号
    
    // 计算总推力向量（EFCS）
    Eigen::Vector3d f_total = mass_ * (a_d - gravity); 
    
    // 转换到机体坐标系（论文公式2）
    Eigen::Vector3d f_b = R_eb.transpose() * f_total;
    
    // 添加对 f_b 的检查，确保其有效
    if (!std::isfinite(f_b.x()) || !std::isfinite(f_b.y()) || !std::isfinite(f_b.z()))
    {
        ROS_ERROR("There are invaild: [%f, %f, %f]", f_b.x(), f_b.y(), f_b.z());
        return;
    }
    
    // 提取 z 分量前先检查
    double f_d = f_b.z();
    
    // 检查 f_d 是否为 NaN 或无穷大
    if (!std::isfinite(f_d))
    {
        ROS_ERROR("The f_d is invaild: %f", f_d);
        // 使用安全默认值
        f_d = max_thrust_ / 2.0;
    }

    f_d = std::max(0.0, std::min(f_d, max_thrust_));
    // f_d = std::max(0.1 * max_thrust_, std::min(f_d, 0.8 * max_thrust_));

    // ------ 姿态控制（论文公式21,26）------
    // 论文公式(21)：计算期望旋转矩阵
    Eigen::Vector3d n_fd = f_total.normalized(); // 期望推力方向（EFCS）
    

    Eigen::Vector3d n_f_current = R_eb.col(2);   // 当前推力方向（BCS Z轴）
    
    // 计算旋转轴和角度
    Eigen::Vector3d r = n_f_current.cross(n_fd);
    double dot = n_f_current.dot(n_fd);
    dot = clamp(dot, -1.0, 1.0); // 确保acos参数有效
    double phi = acos(dot);
    
    // 小角度近似处理
    Eigen::Matrix3d R_lift = Eigen::Matrix3d::Identity();
    if (phi > 1e-6)
    {
        Eigen::Matrix3d skew_r = skewSymmetricMatrix(r.normalized());
        R_lift = Eigen::Matrix3d::Identity() + sin(phi)*skew_r + 
                (1 - cos(phi))*skew_r*skew_r;
    }
    
    // 期望旋转矩阵（论文公式21）
    Eigen::Matrix3d R_d = R_lift * R_eb;

    // 论文公式(26)：姿态误差计算
    Eigen::Matrix3d R_err = R_d.transpose() * R_eb - R_eb.transpose() * R_d;
    Eigen::Vector3d b_omega2 = -0.5 * Eigen::Vector3d(
        R_err(2,1) - R_err(1,2),
        R_err(0,2) - R_err(2,0),
        R_err(1,0) - R_err(0,1)
    );

    // 论文公式(28)：角速度合成与限幅
    Eigen::Vector3d omega_cmd = b_omega1 + b_omega2;
    // Eigen::Vector3d omega_cmd = -(b_omega1 + b_omega2);
    omega_cmd = omega_cmd.cwiseMax(-max_ang_vel_).cwiseMin(max_ang_vel_);

    // ------ MAVROS控制指令发布 ------
    cmd_msg.header.stamp = ros::Time::now();
    
    // 设置归一化推力（0~1）
    cmd_msg.thrust = f_d / max_thrust_;
    
    // 设置机体角速度指令（rad/s）
    cmd_msg.body_rate.x = omega_cmd.x();
    cmd_msg.body_rate.y = omega_cmd.y();
    cmd_msg.body_rate.z = omega_cmd.z();
    
    // 设置控制模式（纯角速度控制）
    cmd_msg.type_mask = 128;
    
    cmd_pub_.publish(cmd_msg);

    // 调试输出
    ROS_INFO("Publishing control: type_mask=%d, thrust=%.2f, body_rate=[%.2f, %.2f, %.2f]", 
        cmd_msg.type_mask, cmd_msg.thrust, 
        cmd_msg.body_rate.x, cmd_msg.body_rate.y, cmd_msg.body_rate.z);
    // ROS_INFO("rel_pos = [%f, %f, %f], dist = %f", rel_pos.x(), rel_pos.y(), rel_pos.z(), rel_dist);
    // ROS_INFO("rel_vel = [%f, %f, %f]", rel_vel.x(), rel_vel.y(), rel_vel.z());
    // ROS_INFO("n_t = [%f, %f, %f], n_td_ = [%f, %f, %f]", n_t.x(), n_t.y(), n_t.z(), n_td_.x(), n_td_.y(), n_td_.z());
    // ROS_INFO("a_d = [%f, %f, %f]", a_d.x(), a_d.y(), a_d.z());
    // ROS_INFO("f_total = [%f, %f, %f]", f_total.x(), f_total.y(), f_total.z());
    // ROS_INFO("f_b = [%f, %f, %f]", f_b.x(), f_b.y(), f_b.z());
    // ROS_INFO("f_d = %f", f_d);

}
