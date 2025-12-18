#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <interface_pkg/match_info.h>
// 发送模拟串口消息

ros::Publisher can_serial_pub;

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "can_node");//初始化test_node节点

    ros::NodeHandle nh;//创建一个节点句柄nh

    interface_pkg::match_info srl;
    srl.bullet = 700;
    srl.middleState = 0;
    srl.resT = 320;
    srl.stage = '4'; // 0 未开始；1 准备；2 自检；3 倒计时； 4 对战中； 5 比赛结束
    // srl.stage = '0'; //测试关于stage的判断逻辑 
    srl.HPself_div10 = 600;
    srl.HPselfmax_div10 = 600;
    srl.HPred_div10 = 60;
    srl.HPblue_div10 = 60;



    //定义发布者，并授予其advertise到指定topic的权限
    can_serial_pub = nh.advertise<interface_pkg::match_info>("/match_info_topic", 10);

    ros::Rate loop_rate(30);//设置循环频率
    while(ros::ok())//创建循环
    {   
        //回泉水测试
        srl.HPself_div10 -= 1;
        ROS_INFO("[can_node]publish: HPself_div10: %d", srl.HPself_div10);
        can_serial_pub.publish(srl);
        ros::spinOnce();//处理回调
        loop_rate.sleep();//按照设定的频率等待
    }
    return 0;
}
