#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <stdlib.h>
#include "image_moment/image_moment.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_moment");
  ros::NodeHandle nh;  
  ImageMoment my_image_moment;
  ros::Rate loop_rate(30.0);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep(); 
  }
}
