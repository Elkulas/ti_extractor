#include"ros/ros.h"

int main(int argc,char **argv)
{
   ros::init(argc,argv,"hello");
   ros::NodeHandle n;
   ros::Rate loop_rate(10);
   while(ros::ok())
   {
      ROS_INFO("Hello World!");
      ros::spinOnce();
      loop_rate.sleep();
   }
   return 0;
}
