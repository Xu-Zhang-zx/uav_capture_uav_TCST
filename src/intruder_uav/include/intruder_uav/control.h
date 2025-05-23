#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <deque>
#include <ncurses.h>
#include <sstream>
#include <algorithm>

class IntruderControl
{
    ros::NodeHandle nh_;

    ros::Publisher vel_pub_;
    ros::Publisher cmd_pub;
    ros::Subscriber pose_sub_;
    ros::Subscriber key_sub_;

    geometry_msgs::Twist cmd_vel_enu;
    geometry_msgs::PoseStamped local_pose;
    geometry_msgs::Twist cmd_twist;

    enum State { 
        IDLE = 0,                // 待机状态 
        OFFBOARD = 1,            // 板外模式就绪状态 
        TAKEOFF = 2,             // 起飞过程
        ESCAPE = 3,              // 开始逃跑 
        HOVERING = 4,            // 悬停状态 
        FOLLOWING_WAYPOINTS = 5, // 航点追踪 
        RETURNING = 6,           // 返航状态
        z_escape = 7,            // z字型逃跑
        sin_escape = 8,          // sin型逃跑
        circle_escape = 9        // 圆形逃跑 
    };

    bool flag_hover = false;

    float Kp = 2;
    double MAX_SPEED = 1.5;
    double cur_x = 0;
    double cur_y = 0;
    double cur_z = 0;
    const double PI = 3.14159265358979323846;
    geometry_msgs::Point set_point;

    std::vector<geometry_msgs::Point> waypoints_;
    size_t current_waypoint_index_;
    State current_state_;

    WINDOW* status_win_;  // 状态显示窗口
    std::deque<std::string> status_messages;
    const int MAX_MSG_LINES = 5;
    std_msgs::String key;

public:
    bool ui_running_ = true;     // 界面运行标志
    IntruderControl():current_waypoint_index_(0), current_state_(IDLE)
    {
        vel_pub_  = nh_.advertise<geometry_msgs::Twist>("/xtdrone/typhoon_h480_0/cmd_vel_enu", 1);
        cmd_pub = nh_.advertise<std_msgs::String>("/xtdrone/typhoon_h480_0/cmd", 3);
        pose_sub_ = nh_.subscribe("typhoon_h480_0/mavros/local_position/pose", 1, &IntruderControl::poseSubscribe, this);
        
        // 初始化用户界面
        init_ui();
    }

    ~IntruderControl() {}

    void velocityControl();
    void poseSubscribe(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // 新增方法
    void init_ui();
    void update_ui();
    void shutdown_ui();
    void process_input(int ch);
    void cal_pid(double dy, double dz, double dx = 0.0);
};