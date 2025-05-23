#ifndef CONTROL_ONLY_H
#define CONTROL_ONLY_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <mavros_msgs/AttitudeTarget.h>



class HighSpeedInterceptionControl
{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber iris_imu_sub_, typhoon_imu_sub_;
    ros::Subscriber iris_pos_sub_, typhoon_pos_sub_;
    ros::Subscriber iris_vel_sub_, typhoon_vel_sub_;

    ros::Publisher cmd_pub_;
    
    // 控制发布话题
    mavros_msgs::AttitudeTarget cmd_msg;
    
    geometry_msgs::PoseStamped iris_pos;
    geometry_msgs::PoseStamped typhoon_pos;
    geometry_msgs::Vector3 iris_vel;
    geometry_msgs::Vector3 typhoon_vel;

    Eigen::Quaterniond q_iris;
    Eigen::Quaterniond q_typhoon;

    // 控制器参数
    double k1_, k2_, k_b_;
    // 无人机质量 kg
    double mass_; 
    // 无人机最大推力 N
    double max_thrust_;
    // 无人机最大角速度 rad/s
    double max_ang_vel_;


    // 初始化相机到机体的旋转矩阵
    Eigen::Matrix3d R_bc = Eigen::Matrix3d::Identity();

    // 机体坐标系下的摄像头光轴方向
    Eigen::Vector3d camera_axis_body_ = Eigen::Vector3d(1, 0, 0);

    // 设计参数
    Eigen::Vector3d n_td_ = Eigen::Vector3d::UnitX(); // 设计视线向量

    public:
    // 初始化
    HighSpeedInterceptionControl();
    void initParams();
    
    void imuCallback_iris(const sensor_msgs::Imu::ConstPtr& msg);
    void imuCallback_typhoon(const sensor_msgs::Imu::ConstPtr& msg);
    
    void posCallback_iris(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void posCallback_typhoon(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    void velCallback_iris(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void velCallback_typhoon(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void updateControl();
};

#endif