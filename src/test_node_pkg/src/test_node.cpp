#include <ros/ros.h>

#include "interface_pkg/serial_from_stm32.h"
#include "interface_pkg/position.h"
#include "interface_pkg/target.h"

#include "map_simulate.hpp"

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#define AgentNumber 1

ros::Publisher serial_pub, target_pub, position_pub, resultPoint_pub;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_;

std::vector<Agent> agents;

visualization_msgs::MarkerArray MarkerArray; // 定义MarkerArray对象

double t;
double t0;

void update()
{
    t = ros::Time::now().sec - t0;
    // 发布当前位置
    // interface_pkg::position posi;
    // posi.x=agents[0].x;
    // posi.y=agents[0].y;
    // posi.yaw=agents[0].yaw;
    // posi.timeStamp=ros::Time::now();
    // position_pub.publish(posi);

    // 设置目标坐标系和源坐标系的名称
    std::string target_frame = "body"; // 目标坐标系
    std::string source_frame = "map";  // 源坐标系

    // 获取 TF 并发布
    tf::TransformListener listener;
    listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));
    tf::StampedTransform transform;
    listener.lookupTransform("map", "body", ros::Time(0), transform);

    interface_pkg::position target_msg;
    target_msg.x = transform.getOrigin().x();
    target_msg.y = transform.getOrigin().y();
    target_msg.yaw = tf::getYaw(transform.getRotation());
    target_msg.timeStamp = ros::Time::now();
    position_pub.publish(target_msg);

    for (int i = 1; i < AgentNumber; i++)
    {
        agentUpdate(agents[i]);
        // 发布target
        interface_pkg::target tar;
        tar.x = agents[i].x;
        tar.y = agents[i].y;
        tar.z = 0;
        tar.type = 1;
        tar.id = i;
        tar.t = t;
        tar.timeStamp = ros::Time::now();
        target_pub.publish(tar);
    }
    // 可视化
    for (int i = 0; i < AgentNumber; i++)
    {
        visualization_msgs::Marker Marker; // 定义Marker对象
        Marker.header.frame_id = "map";
        Marker.type = visualization_msgs::Marker::SPHERE; // 选用文本类型
        Marker.ns = "agents";                             // 必写，否则rviz无法显示
        Marker.id = i;                                    // 用来标记同一帧不同的对象，如果后面的帧的对象少于前面帧的对象，那么少的id将在rviz中残留，所以需要后续的实时更新程序
        Marker.lifetime = ros::Duration(4.0);

        Marker.scale.x = 0.3;
        Marker.scale.y = 0.3;
        Marker.scale.z = 0.3; // 大小

        Marker.color.b = 0;
        Marker.color.g = 1;
        Marker.color.r = 0; // 颜色
        Marker.color.a = 1;
        if (i == 0)
            Marker.color.b = 1;

        Marker.pose.position.x = agents[i].x;
        Marker.pose.position.y = agents[i].y;
        Marker.pose.position.z = 0;

        Marker.pose.orientation.x = 0;
        Marker.pose.orientation.y = 0;
        Marker.pose.orientation.z = 0;
        Marker.pose.orientation.w = 1;

        MarkerArray.markers.push_back(Marker);
    }
    resultPoint_pub.publish(MarkerArray);
    // 发布上行串口数据
    interface_pkg::serial_from_stm32 srl;
    srl.start = 'S';
    srl.end = 'E';
    srl.stage = '4'; // 0 未开始；1 准备；2 自检；3 倒计时； 4 对战中； 5 比赛结束
    srl.resT = 320 - t;
    srl.identity = 1;
    srl.goldenCoin = 0;
    srl.bullet = 700;
    srl.redScore = 0;
    srl.blueScore = 0;
    srl.r1hp = 100;
    srl.r3hp = 100;
    srl.r4hp = 100;
    srl.rshp = 600;
    srl.b1hp = 100;
    srl.b3hp = 100;
    srl.b4hp = 100;
    srl.bshp = 600;
    srl.rbhp = 100;
    srl.bbhp = 1000;
    srl.checksum = 0;
    serial_pub.publish(srl);
    //
    // ROS_INFO("Update !");
    std::string s = "rest Time = " + std::to_string(srl.resT) + "s.";
    ROS_INFO(s.c_str());
}

void onRecvPos(interface_pkg::position p)
{
    // agents[0].yaw = atan2(p.y - agents[0].y, p.x - agents[0].x);
    // float vx = cos(agents[0].yaw), vy = sin(agents[0].yaw);
    // agents[0].x += vx * dt;
    // agents[0].y += vy * dt;

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map"; // Assuming the goal is in the map frame
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = p.x;
    goal.target_pose.pose.position.y = p.y;
    goal.target_pose.pose.orientation.z = sin(p.yaw / 2.0);
    geometry_msgs::Quaternion q=tf::createQuaternionMsgFromRollPitchYaw(0, 0, p.yaw);   
    goal.target_pose.pose.orientation = q;
    // ROS_INFO("__________________________RECV1!!!");
    ac_->sendGoal(goal);
    // ac_->waitForResult();
    // ROS_INFO("__________________________RECV2!!!");
}

int main(int argc, char **argv)
{
    // ROS_ERROR("I am here!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    ros::init(argc, argv, "decision_node");
    ros::NodeHandle nh;

    serial_pub = nh.advertise<interface_pkg::serial_from_stm32>("/serialmsg_Topic", 10);
    position_pub = nh.advertise<interface_pkg::position>("/position_topic", 10);
    target_pub = nh.advertise<interface_pkg::target>("/detect_topic", 10);
    resultPoint_pub = nh.advertise<visualization_msgs::MarkerArray>("/Agents", 5);

    ros::Subscriber targPos_sub;
    targPos_sub = nh.subscribe("/target_position_topic", 2, onRecvPos);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ac.waitForServer();
    ac_ = &ac;

    for (int i = 0; i < AgentNumber; i++)
    {
        Agent a;
        a.x = rand() * 12.0 / RAND_MAX;
        a.y = rand() * 8.0 / RAND_MAX;
        a.yaw = rand() * 2.0 / RAND_MAX;
        if (i == 0)
        {
            a.x = 3;
            a.y = 3;
            a.yaw = 0;
        }
        agents.push_back(a);
    }

    ROS_INFO("__________________________________");
    t0 = ros::Time::now().sec;
    ROS_INFO("decision test ready!!!!!");
    ros::Rate loop_rate(50); // 设置循环频率
    while (ros::ok())
    {
        update(); // 执行循环体内的函数
        // ROS_INFO("____________________________1");
        ros::spinOnce(); // 处理回调函数
        // ROS_INFO("____________________________2");
        loop_rate.sleep(); // 按照设定的频率等待
        // ROS_INFO("____________________________3");
    }

    return 0;
}
