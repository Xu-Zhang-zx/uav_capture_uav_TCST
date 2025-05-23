#include <ros/ros.h>
#include "high_speed_interception/control_only.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "high_speed_interception");
    HighSpeedInterceptionControl controller;
    
    ros::Rate rate(200);
    while(ros::ok())
    {
        controller.updateControl();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}