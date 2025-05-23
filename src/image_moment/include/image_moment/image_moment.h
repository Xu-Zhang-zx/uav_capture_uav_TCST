#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include "image_moment/BoundingBox.h" 
#include "image_moment/BoundingBoxes.h"
#include "image_moment/Moment.h"
#include <algorithm> 

class ImageMoment
{
    ros::NodeHandle nh_;
    
    std::string processing_type_;

    ros::Subscriber imu_sub_;
    ros::Subscriber tag_sub_;
    
    ros::Publisher moment_pub_;
    ros::Publisher info_pub;
    ros::Publisher pixel_pub_;

    // 存储IMU数据
    double pose_data[4][20] = {0.0};
    int record_num = 0;
    // 获取的图像中的位置
    float corner[4][2];
    // 在虚拟平面中的位置
    float corner_virtual[4][2];
    // 图像像素中心
    int center_x = 640, center_y = 360;
    
    // 声明角点坐标变量
    image_moment::BoundingBoxes targets;
    image_moment::BoundingBox target;

    // 图像矩话题参数
    image_moment::Moment moment;
    // 姿态角
    double match_roll, match_pitch, match_yaw;
    float u_ture, v_ture;
    // 图像矩参数
    double bigTagHalfSideLength = 200.0;
    float focal_length = 554.382713;
    double bigTag_mu20_star = 4 * pow(bigTagHalfSideLength, 2);
    double bigTag_mu02_star = 4 * pow(bigTagHalfSideLength, 2);
    double bigTag_a_star = bigTag_mu20_star + bigTag_mu02_star;
    double bigTag_z_star = focal_length/bigTagHalfSideLength;
    double bigTag_qx, bigTag_qy, bigTag_qh, bigTag_qpsi;

    // 像素话题参数
    geometry_msgs::Point center;

    struct Euler
    {
        float roll;
        float pitch;
        float yaw;
    }eu1_;

    struct Quaternion
    {
        float x;
        float y;
        float z;
        float w;
    }qua_;
    
    public:
    ImageMoment(): nh_("~")
    {
        // 初始化

        // 参数解析
        nh_.param<std::string>("type", processing_type_, "moment");

        // 消息发布
        moment_pub_ = nh_.advertise<image_moment::Moment>("/xtdrone/iris_0/image_moment", 1);
        pixel_pub_ = nh_.advertise<geometry_msgs::Point>("/xtdrone/iris_0/center_pixel", 1);

        // 消息订阅
        if (processing_type_ == "moment")
        {
            imu_sub_ = nh_.subscribe("/iris_0/mavros/imu/data", 10, &ImageMoment::getIMUdata, this);
        }
        tag_sub_ = nh_.subscribe("/yolov11/BoundingBoxes", 10, &ImageMoment::getImgdata, this);
        
    }

    // 析构函数
    ~ImageMoment()
    {
    }

    void getIMUdata(const sensor_msgs::ImuConstPtr &msg);
    void getImgdata(const image_moment::BoundingBoxes::ConstPtr& msg);
    void obtain_Virtual_corner(float corner[4][2], double roll, double pitch, float corner_virtual[4][2]);

};
