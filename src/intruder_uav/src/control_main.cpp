#include <ros/ros.h>
#include "intruder_uav/control.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "intruder_uav");
    IntruderControl my_control;
    
    while(ros::ok() && my_control.ui_running_) 
    {
        my_control.velocityControl(); 
        my_control.update_ui(); 
        ros::spinOnce();
        usleep(50000);
    }
    
    // 显式关闭界面和ROS 
    my_control.shutdown_ui(); 
    ros::shutdown();
    return 0;
}