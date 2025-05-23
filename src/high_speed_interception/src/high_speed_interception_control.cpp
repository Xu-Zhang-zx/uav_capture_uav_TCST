#include "high_speed_interception/high_speed_interception_control.h"

template<typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

// 返回向量v的反对称矩阵
Eigen::Matrix3d HighSpeedInterception::skewSymmetricMatrix(const Eigen::Vector3d& v) const
{
    Eigen::Matrix3d S;
    S <<  0,    -v(2),  v(1),
          v(2),  0,    -v(0),
         -v(1),  v(0),  0;
    return S;
}

// 四元数转换为旋转矩阵
Eigen::Matrix3d HighSpeedInterception::quaternionToRotationMatrix(const Eigen::Quaterniond& q) const
{
    return q.normalized().toRotationMatrix();
}

Eigen::Quaterniond HighSpeedInterception::integrateQuaternion(const Eigen::Quaterniond& q_prev, const Eigen::Vector3d& omega, double dt) const
{
    // 计算旋转向量
    Eigen::Vector3d angle_axis_vec = omega * dt;
    // 计算旋转角度
    double angle = angle_axis_vec.norm();
    // 旋转增量四元数
    Eigen::Quaterniond delta_q;
    // 避免除零
    if (angle > 1e-9)
    { 
        delta_q = Eigen::AngleAxisd(angle, angle_axis_vec / angle);
    }
    // 小角度近似
    else
    {
        delta_q = Eigen::Quaterniond(1.0, angle_axis_vec(0) / 2.0, angle_axis_vec(1) / 2.0, angle_axis_vec(2) / 2.0);
        // 保持归一化
        delta_q.normalize();
    }
    // 更新并归一化四元数
    return (q_prev * delta_q).normalized(); 
}

HighSpeedInterception::HighSpeedInterception() : nh_("~"), first_imu_received_(false), dkf_initialized_(false)
{
    R_bc << 0,  0,  1,
           -1,  0,  0,
            0, -1,  0;
        
    // 初始化参数
    initParams();

    // 初始化DKF
    initDelayedKalmanFilter();

    // 订阅IMU和图像话题
    imu_sub_ = nh_.subscribe("/iris_0/mavros/imu/data", 200, 
                            &HighSpeedInterception::imuCallback, this);
    image_sub_ = nh_.subscribe("/xtdrone/iris_0/center_pixel", 10,
                            &HighSpeedInterception::imageCallback, this);

    // 发布控制指令
    cmd_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/iris_0/mavros/setpoint_raw/attitude",10);

}

// 控制器参数初始化
void HighSpeedInterception::initParams()
{
    // 控制器参数
    nh_.param("k1", k1_, 0.8);
    nh_.param("k2", k2_, 10.8);
    nh_.param("k_b", k_b_, 0.2);
    nh_.param("mass", mass_, 1.55);
    nh_.param("max_thrust", max_thrust_, 28.26);
    // nh_.param("max_thrust", max_thrust_, 10.0);
    nh_.param("max_ang_vel", max_ang_vel_, 0.3);

    // 更新DKF噪声参数以匹配Gazebo IMU插件
    // 陀螺仪噪声密度 (rad/s/sqrt(Hz))
    nh_.param("dkf/gyro_noise_density", gyro_noise_density, 0.00018665);
    
    // 陀螺仪随机游走 (rad/s^2/sqrt(Hz))
    nh_.param("dkf/gyro_bias_random_walk", gyro_bias_random_walk, 3.8785e-05);
    
    // 加速度计噪声密度 (m/s^2/sqrt(Hz))
    nh_.param("dkf/accel_noise_density", acc_noise_density, 0.00186);
    
    // 加速度计随机游走 (m/s^3/sqrt(Hz))
    nh_.param("dkf/accel_bias_random_walk", acc_bias_random_walk, 0.006);

    nh_.param("dkf/image_noise_stddev", image_noise_stddev, 1.0); // 像素
    nh_.param("dkf/max_buffer_duration_sec", max_buffer_duration_, 1.0);       // 秒
    nh_.param("dkf/image_pipeline_delay_sec", image_pipeline_delay_sec_, 0.1); // 秒
    
    ROS_INFO("HighSpeedInterception have been initialized");
}

// 初始化DKF
void HighSpeedInterception::initDelayedKalmanFilter()
{
    // 初始化状态向量 x_hat_
    x_hat_.setZero();

    // 初始协方差
    P_.setIdentity();
    // 对位置和速度部分使用较大的初始不确定性
    P_.block<3, 3>(4, 4) = 10.0 * Eigen::Matrix3d::Identity();  // 位置不确定性
    P_.block<3, 3>(7, 7) = 5.0 * Eigen::Matrix3d::Identity();   // 速度不确定性
    // 姿态和零偏使用较小的不确定性
    P_.block<4, 4>(0, 0) = 0.1 * Eigen::Matrix4d::Identity();   // 姿态四元数不确定性
    
    // 基于IMU参数调整零偏的初始不确定性
    // 陀螺仪零偏初始不确定性 - 使用turn-on bias作为初始值
    double gyro_turn_on_bias = 0.0087;  // rad/s
    P_.block<3, 3>(12, 12) = gyro_turn_on_bias * gyro_turn_on_bias * Eigen::Matrix3d::Identity();
    
    // 加速度计零偏初始不确定性
    double accel_turn_on_bias = 0.196;  // m/s^2
    P_.block<3, 3>(15, 15) = accel_turn_on_bias * accel_turn_on_bias * Eigen::Matrix3d::Identity();

    // 过程噪声协方差 Q_process_noise_psd_
    // 这个 Q 是针对 [n_bgyr, n_bacc] 的，它们驱动 db/dt = n
    // 所以对角元素应该是 n_bgyr 和 n_bacc 的PSD (通常作为零偏的随机游走给出)
    Q_process_noise_psd_.setZero();
    Q_process_noise_psd_.diagonal() << 
        Eigen::Vector3d::Constant(gyro_bias_random_walk).array().square(),
        Eigen::Vector3d::Constant(acc_bias_random_walk).array().square();

    // 图像噪声协方差
    R_image_measurement_noise_.setIdentity();
    R_image_measurement_noise_ *= std::pow(image_noise_stddev, 2);

    // 初始化标志
    first_imu_received_ = false;
    first_image_received_ = false;
}


// 处理第一帧IMU数据
void HighSpeedInterception::processFirstImu(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    if (first_imu_received_)
    {
        return;
    }
    
    x_hat_.segment<4>(0) = Eigen::Vector4d(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
    
    // 从IMU获取初始线速度（需要积分加速度）
    // 这里简单起见，假设无人机的速度很小
    Eigen::Vector3d initial_velocity(0.0, 0.0, 0.0);
    
    // 如果知道目标无人机的线速度，可以计算相对速度
    // 这里假设没有先验信息，所以初始相对速度设为零
    x_hat_.segment<3>(7) = initial_velocity;
    
    first_imu_received_ = true;
    last_imu_time_ = imu_msg->header.stamp;
    
    ROS_INFO("First IMU processed, initial velocity set");
}

// 处理第一帧图像
void HighSpeedInterception::processFirstImage(const geometry_msgs::Point::ConstPtr& msg)
{
    if (first_image_received_)
    {
        return;
    }
    
    x_hat_.segment<2>(10) << msg->x, msg->y;

    Eigen::Vector3d pr_initial = Eigen::Vector3d::UnitX() * (-8.0);
    
    // 更新状态中的相对位置
    x_hat_.segment<3>(4) = pr_initial;
    
    first_image_received_ = true;

    ROS_INFO_STREAM("Initial relative position: " << pr_initial.transpose());
    
    ROS_INFO("First Image processed, initial velocity set");
}


// IMU回调
void HighSpeedInterception::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // 处理第一帧IMU数据
    if (!first_imu_received_)
    {
        processFirstImu(msg);
        return;
    }
    // 添加IMU数据到缓存
    imu_buffer_.push_back(*msg);
    
    // 当有足够数据时初始化DKF
    if (!dkf_initialized_ && imu_buffer_.size() >= min_imu_buffer_size_)
    {
        dkf_initialized_ = true;
        ROS_INFO("DKF successfully initialized");
    }
    if (!dkf_initialized_)
    {
        ROS_WARN_THROTTLE(1.0, "DKF not initialized, skipping IMU callback");
        return;
    }
    // 调用DKF预测步骤
    dkfPredict(msg); 
}


// 图像特征回调（带时间戳）
void HighSpeedInterception::imageCallback(const geometry_msgs::Point::ConstPtr& msg) 
{
    // 处理第一帧图像数据
    if (!first_image_received_)
    {
        processFirstImage(msg);
        return;
    }

    // 提取二维坐标
    pixel_center << msg->x, msg->y;
    // 调用处理函数
    processDelayedMeasurement(pixel_center);
}

// DKF预测步骤
void HighSpeedInterception::dkfPredict(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    // 如果是第一次接收IMU数据
    if (!first_imu_received_)
    {
        // 记录当前IMU消息的时间戳作为上一次IMU时间
        last_imu_time_ = imu_msg->header.stamp;
        // 标记已经接收到第一个IMU数据
        first_imu_received_ = true;

        // 将初始状态存入缓存
        DkfState current_buffered_state;
        // 设置缓存状态的时间戳
        current_buffered_state.timestamp = last_imu_time_;
        // 使用当前的全局状态 x_hat_作为缓存状态
        current_buffered_state.x_hat = x_hat_;
        // 使用当前的全局协方差 P_作为缓存协方差
        current_buffered_state.P = P_;
        // 初始状态没有前一个时间步长，所以 dt_input 为 0
        current_buffered_state.dt_input = 0;
        // 初始状态没有应用的IMU输入，设为零
        current_buffered_state.omega_body_input.setZero();
        current_buffered_state.acc_body_input.setZero();
        // 将这个初始状态压入状态缓存队列的末尾
        state_buffer_.push_back(current_buffered_state);
        // 首次处理完成，直接返回
        return;
    }

    // 计算当前IMU消息与上一次IMU消息之间的时间差 (单位：秒)
    // 这对应于卡尔曼滤波器中的时间步长 Δt
    double dt = (imu_msg->header.stamp - last_imu_time_).toSec();

    // 对dt进行有效性检查
    if (dt <= 1e-6 && dt >= 0)
    {
        // 允许非常小的正dt进行标称操作
        // 如果 dt 为零 (例如，收到了重复的IMU消息或时间戳问题)
        if (dt == 0 && !state_buffer_.empty())
        {
            // 更新缓存中最后一个状态的时间戳为当前IMU消息的时间戳
            // 这可以防止在dt为零时，由于时间戳未更新而导致后续dt计算错误
            state_buffer_.back().timestamp = imu_msg->header.stamp;
            // 同时更新全局的 last_imu_time_
            last_imu_time_ = imu_msg->header.stamp;
        }
        // 如果 dt 非常小，预测仍可能运行，但可能很敏感。
        // predictStepInternal 会处理非常小的 dt。
        // 这里的注释表明，即使dt非常小，也会继续执行预测，
        // predictStepInternal内部有对极小dt的处理逻辑。
    } 
    else if (dt < 0)
    { 
        // 如果时间步长为负 (通常由乱序消息引起)
        ROS_ERROR("dt in dkfPredict is negative (dt = %f). Resetting IMU timing", dt);
        // 重置标志，以便下一次IMU消息被当作第一个消息处理
        first_imu_received_ = false;
        // 清空状态缓存，因为历史记录因时间错乱而无效
        state_buffer_.clear();
        // 将 x_hat_ 和 P_ 重新初始化为默认值或最后一个已知的良好值
        // 调用 initDelayedKalmanFilter() 来重置滤波器状态
        initDelayedKalmanFilter();
        // 发生错误，直接返回
        return;
    }

    // --- 获取IMU测量值 ---
    // 从接收到的IMU消息中提取三轴角速度数据，测量的角速度(ω_m)
    Eigen::Vector3d omega_gyr_measured(imu_msg->angular_velocity.x,
                                       imu_msg->angular_velocity.y,
                                       imu_msg->angular_velocity.z);
    
    Eigen::Quaterniond measured_q(imu_msg->orientation.w,
                                  imu_msg->orientation.x,
                                  imu_msg->orientation.y,
                                  imu_msg->orientation.z);

    Eigen::Vector3d acc_body_measured(imu_msg->linear_acceleration.x,
                                      imu_msg->linear_acceleration.y,
                                      imu_msg->linear_acceleration.z);
    
    // 惯性坐标系中的重力向量
    Eigen::Vector3d gravity_inertial(0, 0, -9.81);

    // --- 从先前的状态估计 x_hat_ 中补偿零偏 ---
    // 从上一时刻的状态向量 x_hat_ 中提取陀螺仪零偏的估计值 b_g
    Eigen::Vector3d b_gyr_prev = x_hat_.segment<3>(12);
    // 从上一时刻的状态向量 x_hat_ 中提取加速度计零偏的估计值 b_a
    Eigen::Vector3d b_acc_prev = x_hat_.segment<3>(15);

    // 计算补偿零偏后的机体角速度
    // ω_body_est = ω_m - b_g
    Eigen::Vector3d omega_body_est = omega_gyr_measured - b_gyr_prev;
    // 计算补偿零偏后的机体线加速度
    // acc_body_est = a_m - b_a
    Eigen::Vector3d acc_body_unbiased = acc_body_measured - b_acc_prev;

    // 计算重力在机体坐标系下的表示
    Eigen::Vector3d gravity_body = measured_q.conjugate() * gravity_inertial;

    // 在机体坐标系中补偿重力，得到纯线性加速度
    Eigen::Vector3d acc_body_linear = acc_body_unbiased + gravity_body;

    // 将机体坐标系下的线性加速度转换到惯性坐标系
    Eigen::Vector3d acc_inertial_est = measured_q * acc_body_linear;

    // 打印补偿重力后的加速度
    // ROS_INFO_THROTTLE(0.5, "Gravity-compensated accel: [%.3f, %.3f, %.3f]",
    //     acc_inertial_est.x(), acc_inertial_est.y(), acc_inertial_est.z());

    // --- 核心预测步骤 ---
    // 声明用于存储预测的下一时刻状态的变量 x_hat_{k|k-1}
    Eigen::Matrix<double, 18, 1> x_next_pred;
    // 声明用于存储预测的下一时刻协方差的变量 P_{k|k-1}
    Eigen::Matrix<double, 18, 18> P_next_pred;

    // 调用预测步骤函数
    // 输入:
    //   x_hat_ (即 x_hat_{k-1|k-1}): 上一时刻的后验状态估计
    //   P_ (即 P_{k-1|k-1}): 上一时刻的后验协方差
    //   omega_body_est: 当前时刻补偿零偏后的角速度 (作为输入 u_k 的一部分)
    //   acc_inertial_est: 当前时刻补偿零偏后的线加速度 (作为输入 u_k 的一部分)
    //   dt: 时间步长 Δt
    // 输出:
    //   x_next_pred (即 x_hat_{k|k-1}): 预测的当前状态
    //   P_next_pred (即 P_{k|k-1}): 预测的当前协方差
    //
    // 对应论文中的核心预测方程:
    // 1. 状态预测: x_hat_{k|k-1} = f(x_hat_{k-1|k-1}, u_k, 0)
    //    (其中 f 是非线性状态转移函数, u_k 是输入, 0表示过程噪声均值为零)
    // 2. 协方差预测: P_{k|k-1} = F_k * P_{k-1|k-1} * F_k^T + G_k * Q_psd * G_k^T
    //    (其中 F_k 是状态转移矩阵, G_k 是噪声输入矩阵, Q_psd 是过程噪声功率谱密度)
    if (!predictStepInternal(x_hat_, P_, omega_body_est, acc_inertial_est, measured_q, dt, x_next_pred, P_next_pred))
    {
        // 如果内部预测步骤由于某种原因（例如dt为负导致predictStepInternal返回false）失败
        ROS_WARN("Internal prediction step failed");
        // 保持上一状态不变，不更新 x_hat_ 和 P_
        // 但更新时间戳，以避免使用相同的无效dt重复失败
        last_imu_time_ = imu_msg->header.stamp;
        // 结束本次预测
        return;
    }

    // --- 更新当前全局状态和协方差 ---
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        // 将 predictStepInternal 计算得到的预测状态 x_next_pred 赋值给全局状态变量 x_hat_
        // x_hat_ 此刻代表 x_hat_{k|k-1}
        x_hat_ = x_next_pred;
        // 将 predictStepInternal 计算得到的预测协方差 P_next_pred 赋值给全局协方差变量 P_
        // P_ 此刻代表 P_{k|k-1}
        P_ = P_next_pred;
    }
    // 更新上一次IMU数据的时间戳，用于下一次dt的计算
    last_imu_time_ = imu_msg->header.stamp;

    // --- 将预测的状态存入缓存 ---
    // 创建一个新的 DkfState 结构体实例，用于存储当前预测的结果
    DkfState current_buffered_state;
    // 设置缓存状态的时间戳为当前IMU消息的时间戳
    current_buffered_state.timestamp = last_imu_time_;
    // 存储预测后的状态 x_hat_{k|k-1}
    current_buffered_state.x_hat = x_hat_;
    // 存储预测后的协方差 P_{k|k-1}
    current_buffered_state.P = P_;
    // 存储导致这个预测状态的IMU输入和时间步长。
    // 这些信息在DKF的更新步骤中，当需要从历史状态重新传播时会用到。
    current_buffered_state.omega_body_input = omega_body_est; // 记录用于这次预测的角速度输入
    current_buffered_state.acc_body_input = acc_inertial_est;     // 记录用于这次预测的加速度输入
    current_buffered_state.measured_q = measured_q;
    current_buffered_state.dt_input = dt;                     // 记录用于这次预测的时间步长
    // 将这个包含最新预测状态的 DkfState 对象添加到状态缓存队列的末尾
    state_buffer_.push_back(current_buffered_state);

    // --- 修剪缓存 (移除过旧的状态) ---
    // 检查状态缓存队列是否非空，并且最早的状态的时间戳与当前时间戳的差值是否超过了设定的最大缓存时长
    while (!state_buffer_.empty() && (last_imu_time_ - state_buffer_.front().timestamp).toSec() > max_buffer_duration_)
    {
        // 如果缓存中最老的状态超出了 max_buffer_duration_，则将其从队列头部移除
        state_buffer_.pop_front();
    }
    // 这个循环确保了状态缓存的大小不会无限增长，只保留最近一段时间内的状态历史。
    // max_buffer_duration_ 需要大于等于图像处理的延迟，以便在收到图像数据时能找到对应的历史状态。
}

bool HighSpeedInterception::predictStepInternal(
    const Eigen::Matrix<double, 18, 1>& x_prev,     // 上一时刻状态估计 (x_{k-1|k-1})
    const Eigen::Matrix<double, 18, 18>& P_prev,    // 上一时刻协方差 (P_{k-1|k-1})
    const Eigen::Vector3d& omega_body_est,          // 减去误差的估计的机体角速度 (ω_b,est^b = ω_b^m - b_g)
    const Eigen::Vector3d& acc_inertial_est,        // 减去误差的估计的机体线加速度 (f_b,est^b = a_b^m - b_a)
    const Eigen::Quaterniond& measured_q,
    double dt,                                      // 时间步长 (Δt)
    Eigen::Matrix<double, 18, 1>& x_next,           // 输出：预测的下一时刻状态 (x_{k|k-1})
    Eigen::Matrix<double, 18, 18>& P_next)          // 输出：预测的下一时刻协方差 (P_{k|k-1})
{
    // --- dt 有效性检查 ---
    if (dt <= 1e-6)
    {
        ROS_WARN_THROTTLE(1.0, "dt in predictStepInternal is too small (dt = %f), skipping prediction", dt);
        if (dt < 0)
        {
            return false;
        }
        x_next = x_prev;
        P_next = P_prev + Eigen::Matrix<double, 18, 18>::Identity() * 1e-9;
        return true;
    }

    // --- 构建状态转移矩阵 F_k ---
    Eigen::Matrix<double, 18, 18> Fk = Eigen::Matrix<double, 18, 18>::Identity();

    // 1. 四元数部分 (0-3)
    // F_qq: 四元数到四元数的转移
    const Eigen::Vector3d angle_axis_vec = omega_body_est * dt / 2;
    Eigen::Matrix4d F_qq;
    F_qq << 1, -angle_axis_vec.x(), -angle_axis_vec.y(), -angle_axis_vec.z(),
            angle_axis_vec.x(), 1, angle_axis_vec.z(), -angle_axis_vec.y(),
            angle_axis_vec.y(), -angle_axis_vec.z(), 1, -angle_axis_vec.x(),
            angle_axis_vec.z(), angle_axis_vec.y(), -angle_axis_vec.x(), 1;
    Fk.block<4,4>(0,0) = F_qq;

    // F_q_bgyr: 陀螺仪偏置对四元数的影响
    const Eigen::Quaterniond q_prev(x_prev(0), x_prev(1), x_prev(2), x_prev(3));
    Eigen::Matrix<double, 4, 3> Xi_q;
    Xi_q << -q_prev.x(), -q_prev.y(), -q_prev.z(),
             q_prev.w(), -q_prev.z(),  q_prev.y(),
             q_prev.z(),  q_prev.w(), -q_prev.x(),
            -q_prev.y(),  q_prev.x(),  q_prev.w();
    Fk.block<4,3>(0,12) = -0.5 * Xi_q * dt;

    // 2. 位置部分 (4-6)
    // F_pr_vr: 速度对位置的影响
    Fk.block<3,3>(4,7) = Eigen::Matrix3d::Identity() * dt;

    // 3. 速度部分 (7-9)
    // F_vr_q: 四元数对速度的影响
    const Eigen::Matrix3d R_eb = quaternionToRotationMatrix(q_prev);
    Eigen::Matrix<double, 3, 4> F_vr_q = Eigen::Matrix<double, 3, 4>::Zero();

    // 使用四元数分量
    double q0 = q_prev.w();  // 实部
    double q1 = q_prev.x();  // 虚部i
    double q2 = q_prev.y();  // 虚部j
    double q3 = q_prev.z();  // 虚部k

    Eigen::Matrix<double, 4, 3> M1, M2, M3;

    M1 << q0, -q3,  q2,
          q1,  q2,  q3,
         -q2,  q1,  q0,
         -q3, -q0,  q1;

    M2 << q3,  q0, -q1,
          q2, -q1, -q0,
          q1,  q2,  q3,
          q0, -q3,  q2;

    M3 << -q2,  q1,  q0,
           q3,  q0, -q1,
           q0,  q3, -q2,
           q1,  q2,  q3;
    
    // 计算 F_vr_q
    F_vr_q.row(0) = 2 * (M1 * acc_inertial_est).transpose();
    F_vr_q.row(1) = 2 * (M2 * acc_inertial_est).transpose();
    F_vr_q.row(2) = 2 * (M3 * acc_inertial_est).transpose();
    
    Fk.block<3,4>(7,0) = F_vr_q * dt;

    // F_vr_bacc: 加速度计偏置对速度的影响
    Fk.block<3,3>(7,15) = -R_eb * dt; 

    // 4. 图像特征部分 (10-11)
    // 提取参数
    const Eigen::Vector2d i_p_bar = x_prev.segment<2>(10);

    // 计算p_zc前的安全检查
    Eigen::Vector3d p_c = -R_bc.transpose() * x_prev.segment<3>(4);
    double p_zc = p_c.z();
    
    // 防止p_zc过小导致数值不稳定
    if (std::abs(p_zc) < 1e-3)
    {
        ROS_WARN_THROTTLE(1.0, "p_zc is dangerously small: %f, setting to safe minimum", p_zc);
        p_zc = (p_zc >= 0) ? 1e-3 : -1e-3;
    }

    // 构建图像雅可比矩阵L_s
    Eigen::Matrix<double, 2, 6> L_s;
    L_s << -1/p_zc, 0,       i_p_bar.x()/p_zc, i_p_bar.x()*i_p_bar.y(), -(1 + i_p_bar.x()*i_p_bar.x()), i_p_bar.y(),
           0,      -1/p_zc,  i_p_bar.y()/p_zc, 1 + i_p_bar.y()*i_p_bar.y(), -i_p_bar.x()*i_p_bar.y(),  -i_p_bar.x();

    // 分解线速度和角速度分量
    const Eigen::Matrix<double, 2, 3> L_v = L_s.leftCols(3);
    const Eigen::Matrix<double, 2, 3> L_omega = L_s.rightCols(3);

    // 计算雅可比分量
    const Eigen::Matrix<double, 2, 3> J_v = L_v * R_bc.transpose() * R_eb.transpose();
    const Eigen::Matrix<double, 2, 3> J_omega = -L_omega * R_bc.transpose();

    // F_pi_q: 四元数对图像特征的影响
    Eigen::Matrix<double, 2, 4> F_pi_q;
    // 根据附录C计算M4和M5矩阵
    double px = i_p_bar.x(), py = i_p_bar.y();

    Eigen::Matrix<double, 4, 3> M4;
    M4 << 2*px*q0 + 2*q3,   2*px*q3 - 2*q0,   -2*px*q2 - 2*q1,
          2*px*q1 - 2*q2,   2*px*q2 + 2*q1,    2*px*q3 - 2*q0,
          2*px*q2 - 2*q1,   2*px*q1 - 2*q2,   -2*px*q0 - 2*q3,
          2*px*q3 + 2*q0,   2*px*q0 + 2*q3,    2*px*q1 - 2*q2;

    Eigen::Matrix<double, 4, 3> M5;
    M5 << 2*py*q0 - 2*q2,   2*py*q3 + 2*q1,   -2*py*q2 - 2*q0,
          2*py*q1 - 2*q3,   2*py*q2 + 2*q0,    2*py*q3 + 2*q1,
          2*py*q2 - 2*q0,   2*py*q1 - 2*q3,   -2*py*q0 + 2*q2,
          2*py*q3 - 2*q1,   2*py*q0 - 2*q2,    2*py*q1 - 2*q3;

    F_pi_q.row(0) = (1.0/p_zc) * (M4*x_prev.segment<3>(7)).transpose() * dt;  // 对应x坐标
    F_pi_q.row(1) = (1.0/p_zc) * (M5*x_prev.segment<3>(7)).transpose() * dt;  // 对应y坐标

    Fk.block<2,4>(10,0) = F_pi_q;

    // F_pi_vr: 速度对图像特征的影响
    Fk.block<2,3>(10,7) = J_v * dt;

    // F_pi_pi: 图像特征的状态转移
    Eigen::Matrix2d A;

    // 将角速度从机体坐标系转换到相机坐标系
    Eigen::Vector3d omega_cam = R_bc.transpose() * omega_body_est;
    double omega_xc = omega_cam(0);
    double omega_yc = omega_cam(1);
    double omega_zc = omega_cam(2);

    // 获取相机z方向速度
    Eigen::Vector3d v_cam = R_bc.transpose() * x_prev.segment<3>(7);
    double v_zc = v_cam(2);

    // 构建矩阵A，严格按照论文公式
    A << (v_zc/p_zc) + i_p_bar.y()*omega_xc - 2*i_p_bar.x()*omega_yc, i_p_bar.x()*omega_xc + omega_zc,
         -i_p_bar.y()*omega_yc - omega_zc, (v_zc/p_zc) + 2*i_p_bar.y()*omega_xc - i_p_bar.x()*omega_yc;
    // 计算状态转移矩阵
    Fk.block<2,2>(10,10) = Eigen::Matrix2d::Identity() + A * dt;

    // F_pi_bgyr: 陀螺仪偏置对图像特征的影响
    Fk.block<2,3>(10,12) = J_omega * dt;

    // 5. 偏置部分 (12-17)
    // 陀螺仪偏置
    Fk.block<3,3>(12,12) = Eigen::Matrix3d::Identity();
    // 加速度计偏置
    Fk.block<3,3>(15,15) = Eigen::Matrix3d::Identity();

    // ================== 噪声矩阵G_k ==================
    Eigen::Matrix<double, 18, 6> Gk = Eigen::Matrix<double, 18, 6>::Zero();
    // 陀螺仪噪声
    Gk.block<4,3>(0,0) = -0.5 * Xi_q * dt; 
    // 加速度计噪声
    Gk.block<3,3>(7,3) = -R_eb.transpose() * dt;
    // 图像特征噪声
    Gk.block<2,3>(10,0) = J_omega * dt;

    // ================== 协方差预测 ==================
    P_next = Fk * P_prev * Fk.transpose() + Gk * Q_process_noise_psd_ * Gk.transpose() * dt;

    // ================== 状态预测 ==================
    // 初始化状态预测
    x_next = x_prev;

    // 1. 四元数更新 (使用精确的旋转积分)
    // 计算角速度角增量
    const double angle = angle_axis_vec.norm();
    Eigen::Quaterniond delta_q;
    if (angle > 1e-6)
    {
        // 对于较大的角度，使用角轴表示法更准确
        Eigen::Vector3d axis = angle_axis_vec / angle;
        double half_angle = 0.5 * angle;
        double sin_half_angle = sin(half_angle);
        
        delta_q.w() = cos(half_angle);
        delta_q.x() = axis.x() * sin_half_angle;
        delta_q.y() = axis.y() * sin_half_angle;
        delta_q.z() = axis.z() * sin_half_angle;
    }
    else
    {
        // 对于很小的角度，使用一阶泰勒展开更稳定
        delta_q.w() = 1.0;
        delta_q.x() = angle_axis_vec.x() * 0.5;
        delta_q.y() = angle_axis_vec.y() * 0.5;
        delta_q.z() = angle_axis_vec.z() * 0.5;
        delta_q.normalize();
    }
    // 四元数更新
    Eigen::Quaterniond q_next = delta_q * q_prev;
    q_next.normalize();
    x_next.segment<4>(0) = Eigen::Vector4d(q_next.w(), q_next.x(), q_next.y(), q_next.z());
    
    // x_next.segment<4>(0) = Eigen::Vector4d(measured_q.w(), measured_q.x(), measured_q.y(), measured_q.z());

    // 2. 位置更新
    x_next.segment<3>(4) += x_prev.segment<3>(7) * dt;

    // 3. 速度更新
    Eigen::Vector3d acc_inertial = R_eb * acc_inertial_est;
    x_next.segment<3>(7) += acc_inertial * dt;

    // 4. 图像特征更新
    const Eigen::Vector2d feature_rate = J_v * x_prev.segment<3>(7) + J_omega * omega_body_est;
    x_next.segment<2>(10) += feature_rate * dt;

    // 5. 偏置保持随机游走模型

    return true;
}


// DKF校正步骤（处理延迟测量）
void HighSpeedInterception::processDelayedMeasurement(const Eigen::Vector2d& img_feature)
{
    if (!dkf_initialized_ || !first_imu_received_ || state_buffer_.empty())
    {
        ROS_WARN_THROTTLE(1.0, "DKF not initialized or state cache is empty, unable to process delayed measurements");
        return;
    }

    // ===== 1. 获取延迟测量对应的时间戳 =====
    const ros::Time measurement_time = ros::Time::now() - ros::Duration(image_pipeline_delay_sec_);
    
    // ===== 2. 查找对应的历史状态 =====
    auto target_state_iter = state_buffer_.begin();
    bool found_state = false;
    
    // 遍历状态缓存寻找最接近测量时间的状态
    for (; target_state_iter != state_buffer_.end(); ++target_state_iter)
    {
        if (target_state_iter->timestamp >= measurement_time)
        {
            found_state = true;
            // 如果不是第一个状态且当前状态时间戳大于测量时间，可能前一个状态更接近
            if (target_state_iter != state_buffer_.begin() && 
                (target_state_iter->timestamp - measurement_time).toSec() > 
                (measurement_time - (target_state_iter-1)->timestamp).toSec())
            {
                --target_state_iter; // 使用前一个时间更接近的状态
            }
            break;
        }
    }
    
    // 边界情况处理：如果没有找到有效状态
    if (!found_state)
    {
        if (state_buffer_.empty())
        {
            ROS_WARN_THROTTLE(1.0, "State cache is empty, unable to process delayed measurements");
            return;
        }
        // 使用最早的状态
        target_state_iter = state_buffer_.begin();
        ROS_WARN_THROTTLE(1.0, "Measurement timestamp %.3f is earlier than all cached states, using the earliest state %.3f", 
                         measurement_time.toSec(), target_state_iter->timestamp.toSec());
    }

    // ===== 3. 提取延迟状态和协方差 =====
    DkfState& delayed_state = *target_state_iter;
    const Eigen::Vector2d z_measured = img_feature; // 当前图像测量值
    
    // ===== 4. 构建测量矩阵 H =====
    // 根据论文公式(33)：测量矩阵直接从状态向量中提取图像特征部分
    Eigen::Matrix<double, 2, 18> H = Eigen::Matrix<double, 2, 18>::Zero();
    H.block<2,2>(0, 10) = Eigen::Matrix2d::Identity(); // 直接观测图像特征状态
    
    // ===== 5. 计算残差 =====
    Eigen::Vector2d z_pred = delayed_state.x_hat.segment<2>(10);  // 预测的图像特征
    Eigen::Vector2d residual = z_measured - z_pred;               // 测量残差
    
    // 限制残差大小，防止异常值导致滤波器不稳定
    const double max_residual = 100.0; // 像素
    for (int i = 0; i < 2; ++i)
    {
        if (std::abs(residual(i)) > max_residual)
        {
            residual(i) = (residual(i) > 0) ? max_residual : -max_residual;
            ROS_WARN_THROTTLE(1.0, "Residual is too large, limited to ±%.1f pixels", max_residual);
        }
    }
    
    // ===== 6. 计算创新协方差和卡尔曼增益 =====
    Eigen::Matrix2d S = H * delayed_state.P * H.transpose() + R_image_measurement_noise_;
    
    // 确保S是正定的
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigenSolver(S);
    if (eigenSolver.eigenvalues().minCoeff() < 1e-6)
    {
        S += 1e-6 * Eigen::Matrix2d::Identity();
        ROS_WARN_THROTTLE(1.0, "The innovation covariance matrix is close to singular, a regularization term has been added");
    }
    
    // 计算卡尔曼增益
    Eigen::Matrix<double, 18, 2> K = delayed_state.P * H.transpose() * S.inverse();
    
    // ===== 7. 状态和协方差更新 =====
    // 更新状态估计（论文公式37）
    delayed_state.x_hat += K * residual;
    
    // 归一化四元数部分
    Eigen::Quaterniond q_updated(delayed_state.x_hat(0), delayed_state.x_hat(1), 
                                delayed_state.x_hat(2), delayed_state.x_hat(3));
    q_updated.normalize();
    delayed_state.x_hat.segment<4>(0) << q_updated.w(), q_updated.x(), q_updated.y(), q_updated.z();
    
    // 更新协方差矩阵（论文公式38）
    // 使用Joseph形式以保证数值稳定性和正定性
    Eigen::Matrix<double, 18, 18> I_KH = Eigen::Matrix<double, 18, 18>::Identity() - K * H;
    delayed_state.P = I_KH * delayed_state.P * I_KH.transpose() + K * R_image_measurement_noise_ * K.transpose();
    
    // ===== 8. 从修正的历史状态向前传播 =====
    size_t propagation_count = 0;
    auto current_state_iter = state_buffer_.end() - 1;
    
    for (auto iter = target_state_iter; iter < current_state_iter; ++iter)
    {
        DkfState& current = *iter;
        DkfState& next = *(iter + 1);
        
        // 使用存储的IMU输入重新预测（论文公式39-41）
        Eigen::Matrix<double, 18, 1> x_propagated;
        Eigen::Matrix<double, 18, 18> P_propagated;
        
        bool prediction_success = predictStepInternal(
            current.x_hat, 
            current.P,
            next.omega_body_input,
            next.acc_body_input,
            next.measured_q,
            next.dt_input,
            x_propagated,
            P_propagated
        );
        
        if (prediction_success)
        {
            next.x_hat = x_propagated;
            next.P = P_propagated;
            propagation_count++;
        }
        else
        {
            ROS_WARN("Prediction step failed during forward propagation, keeping the state unchanged");
        }
    }
    
    // ===== 9. 更新全局状态估计 =====
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        x_hat_ = state_buffer_.back().x_hat;
        P_ = state_buffer_.back().P;
    }
    
    ROS_DEBUG("Successfully processed delayed measurement: found the corresponding historical state, propagated forward %zu steps", propagation_count);
}


// 核心控制算法
void HighSpeedInterception::updateControl()
{
    if ( !first_image_received_ || !first_imu_received_)
    {
        ROS_INFO_THROTTLE(1.0, "Waiting for data initialization before starting control");
        return;
    }
    if (state_buffer_.empty())
    {
        ROS_WARN_THROTTLE(1.0, "The state cache is empty, control not executed");
        return;
    }

    Eigen::Matrix<double, 18, 1> current_state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state = x_hat_;
    }

    // ------ 打印DKF状态向量 ------
    // printStateVector();

    // ------ 从状态估计中获取必要参数 ------
    Eigen::Quaterniond q_est(current_state(0), current_state(1), current_state(2), current_state(3)); // BCS->EFCS
    Eigen::Vector3d rel_pos = current_state.segment<3>(4);    // EFCS中的相对位置 p_r
    Eigen::Vector3d rel_vel = current_state.segment<3>(7);    // EFCS中的相对速度 v_r
    

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
    Eigen::Matrix3d R_eb = q_est.toRotationMatrix();
    Eigen::Vector3d n_td_world = R_eb * camera_axis_body_;
    n_td_ = n_td_world.normalized();

    // 论文公式(4)：目标单位向量计算
    Eigen::Vector3d n_t = -rel_pos.normalized(); // EFCS中的LOS向量
    
    // double px = current_state(10);    // 第10位为x
    // double py = current_state(11);    // 第11位为y
    // Eigen::Vector3d cam_vec(px, py, 1); 
    // // 坐标系转换：相机 -> 机体 -> EFCS
    // Eigen::Vector3d ecef_vec = R_eb * R_bc * cam_vec;
    // Eigen::Vector3d n_t = ecef_vec.normalized(); // EFCS中的LOS向量

    // 论文公式(5)：视线跟踪误差
    double z1 = 1.0 - n_td_.dot(n_t);
    
    ROS_INFO("LOS error = %.3f", z1);

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
    // const double max_barrier_term = 100.0;
    // barrier_term = std::min(barrier_term, max_barrier_term);
    
    // 论文公式(13)：角速度控制项
    Eigen::Vector3d b_omega1 = barrier_term * R_eb.transpose() * n_td_.cross(n_t);

    // Eigen::Vector3d b_omega1 = barrier_term * R_eb.transpose() * n_t.cross(n_td_);

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
    Eigen::Vector3d damping = -0.2 * rel_vel;
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
    ROS_INFO("rel_pos = [%f, %f, %f], dist = %f", rel_pos.x(), rel_pos.y(), rel_pos.z(), rel_dist);
    // ROS_INFO("rel_vel = [%f, %f, %f]", rel_vel.x(), rel_vel.y(), rel_vel.z());
    // ROS_INFO("n_t = [%f, %f, %f], n_td_ = [%f, %f, %f]", n_t.x(), n_t.y(), n_t.z(), n_td_.x(), n_td_.y(), n_td_.z());
    // ROS_INFO("a_d = [%f, %f, %f]", a_d.x(), a_d.y(), a_d.z());
    // ROS_INFO("f_total = [%f, %f, %f]", f_total.x(), f_total.y(), f_total.z());
    // ROS_INFO("f_b = [%f, %f, %f]", f_b.x(), f_b.y(), f_b.z());
    // ROS_INFO("f_d = %f", f_d);
}

// 添加状态向量打印函数
void HighSpeedInterception::printStateVector()
{
    // 打印精简版本的状态信息(使用INFO_THROTTLE减少日志频率)
    ROS_INFO_THROTTLE(0.5, "DKF State: q=[%.3f, %.3f, %.3f, %.3f] pos=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f]",
        x_hat_(0), x_hat_(1), x_hat_(2), x_hat_(3),  // 四元数
        x_hat_(4), x_hat_(5), x_hat_(6),            // 相对位置
        x_hat_(7), x_hat_(8), x_hat_(9));           // 相对速度
    
    // 打印图像特征和偏置项(使用更低的频率)
    ROS_INFO_THROTTLE(0.5, "DKF Features: img=[%.3f, %.3f] gyro_bias=[%.5f, %.5f, %.5f] acc_bias=[%.5f, %.5f, %.5f]",
        x_hat_(10), x_hat_(11),                     // 图像特征
        x_hat_(12), x_hat_(13), x_hat_(14),         // 陀螺仪偏置
        x_hat_(15), x_hat_(16), x_hat_(17));        // 加速度计偏置
    
    // 打印状态协方差对角线元素(低频率)
    ROS_DEBUG_THROTTLE(5.0, "DKF Covariance Diag (position): [%.3e, %.3e, %.3e]", 
        P_(4,4), P_(5,5), P_(6,6));
    ROS_DEBUG_THROTTLE(5.0, "DKF Covariance Diag (velocity): [%.3e, %.3e, %.3e]", 
        P_(7,7), P_(8,8), P_(9,9));
    
    // 打印状态有效性检查
    bool has_invalid = false;
    for (int i = 0; i < x_hat_.size(); ++i) {
        if (!std::isfinite(x_hat_(i))) {
            ROS_ERROR("Invalid state element at index %d: %f", i, x_hat_(i));
            has_invalid = true;
        }
    }
    
    // 打印四元数规范性检查
    Eigen::Quaterniond q_check(x_hat_(0), x_hat_(1), x_hat_(2), x_hat_(3));
    double q_norm = q_check.norm();
    if (std::abs(q_norm - 1.0) > 0.01) {
        ROS_WARN("Quaternion not normalized, norm = %.4f", q_norm);
    }
    
    // 如果存在任何无效状态，打印整个状态向量(仅在发现问题时)
    if (has_invalid) {
        std::stringstream ss;
        ss << "Full state vector: [";
        for (int i = 0; i < x_hat_.size(); ++i) {
            ss << x_hat_(i);
            if (i < x_hat_.size() - 1) ss << ", ";
        }
        ss << "]";
        ROS_ERROR_STREAM(ss.str());
    }
}
