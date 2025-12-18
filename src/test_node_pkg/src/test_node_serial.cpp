#include <ros/ros.h>

#include "interface_pkg/serial_cmdmsgs.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_node");
  ros::NodeHandle nh,nh_p("~");
  
  std::string p1,p2;

  
  std::string vel_sub_topic,cmd_sub_topic;

  nh_p.getParam("vel_sub_topic", vel_sub_topic);
  nh_p.getParam("cmd_sub_topic", cmd_sub_topic);

  ros::Publisher vel_pub=nh.advertise<geometry_msgs::Twist>(vel_sub_topic,10);
  ros::Publisher cmd_pub=nh.advertise<interface_pkg::serial_cmdmsgs>(cmd_sub_topic,10);

  ros::Rate loop(50);
  while(ros::ok())
  {
    geometry_msgs::Twist vel;
    interface_pkg::serial_cmdmsgs cmd;

    vel.linear.x=1.0;
    vel.angular.z=0.5;

    cmd.yaw_down=-0.8;

    vel_pub.publish(vel);
    cmd_pub.publish(cmd);

    loop.sleep();
  }
  return 0;
}
