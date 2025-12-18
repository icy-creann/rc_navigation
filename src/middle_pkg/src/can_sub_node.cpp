#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <interface_pkg/serial_from_stm32.h>
// 模拟订阅串口消息
ros::Subscriber serial_msg_sub;

void serial_callback(const interface_pkg::serial_from_stm32::ConstPtr& msg)
{
    ROS_INFO("Serial_msg: %c %c %d %c ", msg->start, msg->stage, msg->resT, msg->identity);
}
int main(int argc, char **argv)
{   
    ROS_INFO("can_sub_node");
    ros::init(argc, argv, "can_sub_node");//初始化can_sub_node节点

    ros::NodeHandle nh;//创建一个节点句柄nh

    //定义订阅者，并订阅指定的topic，当有消息发布到该topic时，调用回调函数
    serial_msg_sub = nh.subscribe<interface_pkg::serial_from_stm32>("/serialmsg_Topic", 10, serial_callback);

    ros::Rate loop_rate(50);//设置循环频率
    while(ros::ok())//创建循环
    {
        ros::spinOnce();//处理回调
        loop_rate.sleep();//按照设定的频率等待
    }
    return 0;
}
