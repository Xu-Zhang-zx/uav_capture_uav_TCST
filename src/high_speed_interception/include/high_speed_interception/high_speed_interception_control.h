#ifndef HIGH_SPEED_INTERCEPTION_CONTROL_H
#define HIGH_SPEED_INTERCEPTION_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <deque>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf/transform_datatypes.h> 
#include <mutex> 


// DKF 状态结构体，用于缓存
struct DkfState 
{
    ros::Time timestamp;                    // 时间戳
    Eigen::Matrix<double, 18, 1> x_hat;     // 状态估计
    Eigen::Matrix<double, 18, 18> P;        // 状态协方差

    // 导致从缓存中前一个状态得到当前状态的输入
    Eigen::Vector3d omega_body_input;       // 估计的机体角速度
    Eigen::Vector3d acc_body_input;         // 估计的机体线加速度
    Eigen::Quaterniond measured_q;
    double dt_input;                        // 时间步长
};

class HighSpeedInterception
{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_, image_sub_;
    ros::Publisher cmd_pub_;
    
    // 控制发布话题
    mavros_msgs::AttitudeTarget cmd_msg;
    
    // 控制器参数
    double k1_, k2_, k_b_;
    // 无人机质量 kg
    double mass_; 
    // 无人机最大推力 N
    double max_thrust_;
    // 无人机最大角速度 rad/s
    double max_ang_vel_;

    // DKF 状态变量
    Eigen::Matrix<double, 18, 1> x_hat_; // 当前状态向量 [q(w,x,y,z), p_r, v_r, p_i, b_gyr, b_acc]
    Eigen::Matrix<double, 18, 18> P_;    // 当前协方差矩阵

    // DKF 参数
    // 陀螺仪和加速度计噪声功率谱密度值
    double gyro_noise_density, acc_noise_density;
    // 陀螺仪和加速度计零偏随机游走功率谱密度值
    double gyro_bias_random_walk, acc_bias_random_walk;
    // 图像测量噪声
    bool first_image_received_;
    double image_noise_stddev;
    Eigen::Vector2d pixel_center = Eigen::Vector2d::Zero();

    float focal_length = 554.382713;

    Eigen::Matrix<double, 6, 6> Q_process_noise_psd_;      // 过程噪声协方差
    Eigen::Matrix2d R_image_measurement_noise_;            // 测量噪声协方差

    ros::Time last_imu_time_;         // 上一次IMU数据的时间戳
    bool first_imu_received_;         // 是否已收到第一个IMU数据
    bool dkf_initialized_;            // DKF是否已初始化
    size_t min_imu_buffer_size_ = 10; // 初始化所需的最小IMU数据量
    std::deque<sensor_msgs::Imu> imu_buffer_;

                                         
    // DKF 状态缓存
    std::deque<DkfState> state_buffer_;     // 用于DKF的状态缓存队列
    double max_buffer_duration_;     // 状态在缓存中保留的最长持续时间
    double image_pipeline_delay_sec_;       // 估计的图像管道延迟

    // 初始化相机到机体的旋转矩阵（从相机坐标系[右下前]到机体坐标系[前左上]）
    Eigen::Matrix3d R_bc;

    // 机体坐标系下的摄像头光轴方向
    Eigen::Vector3d camera_axis_body_ = Eigen::Vector3d(1, 0, 0);
    // 设计参数
    Eigen::Vector3d n_td_ = -Eigen::Vector3d::UnitX(); // 设计视线向量

    std::mutex state_mutex_;

    public:
    // 初始化
    HighSpeedInterception();
    void initParams();
    void initDelayedKalmanFilter();
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void imageCallback(const geometry_msgs::Point::ConstPtr& msg);
    
    // ------DKF------
    // DKF预测步骤
    void dkfPredict(const sensor_msgs::Imu::ConstPtr& imu_msg);

    // 核心预测逻辑，由 dkfPredict 和重传播过程调用
    bool predictStepInternal(const Eigen::Matrix<double, 18, 1>& x_prev,    // 上一时刻状态
                             const Eigen::Matrix<double, 18, 18>& P_prev,   // 上一时刻协方差
                             const Eigen::Vector3d& omega_body_est,         // 估计的机体角速度 (已补偿零偏)
                             const Eigen::Vector3d& acc_body_est,           // 估计的机体线加速度 (比力, 已补偿零偏)
                             const Eigen::Quaterniond& measured_q,
                             double dt,                                     // 时间步长
                             Eigen::Matrix<double, 18, 1>& x_next,          // 输出：下一时刻预测状态
                             Eigen::Matrix<double, 18, 18>& P_next);        // 输出：下一时刻预测协方差
    // DKF更新步骤
    void processDelayedMeasurement(const Eigen::Vector2d& img_feature);

    // 辅助函数
    // 计算向量的反对称矩阵
    Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d& v) const;
    // 四元数转旋转矩阵
    Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& q) const;
    // 四元数积分
    Eigen::Quaterniond integrateQuaternion(const Eigen::Quaterniond& q_prev, const Eigen::Vector3d& omega, double dt) const;

    void processFirstImu(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void processFirstImage(const geometry_msgs::Point::ConstPtr& msg);

    void updateControl();

    void printStateVector();
};

#endif