#include "fuzzy_pid/fuzzy_control.h"
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fuzzy_control");
    ros::NodeHandle nh;  
    FuzzyControl my_fuzzy_control;
    ros::Rate loop_rate(200.0);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep(); 
    }
}