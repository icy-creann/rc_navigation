#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include <vector>

struct Vector3
{
    float x;
    float y;
    float z;
};

#define PI 3.14159265358979323846
// 计算两个角度之间的差值，取劣角部分的度数
float subAngle(float a1, float a2)
{
    // 将角度转换到 -π 到 π 的范围内
    float a1_ = std::fmod((a1 + PI), (2 * PI)) - PI;
    float a2_ = std::fmod((a2 + PI), (2 * PI)) - PI;

    // 计算绝对角度差
    float diff = std::abs(a1_ - a2_);

    // 如果角度差大于 π，则用 2π 减去角度差
    return diff > PI ? (2 * PI - diff) : diff;
}

// 目标位姿数组
int point_index = 0;
std::vector<Vector3> target_poses;

// 距离阈值
Vector3 threshold;

// 发布者
ros::Publisher move_base_target_publisher;

int count=0;

// 回调函数，用于检查机器人当前位置并更新目标
void checkAndPublishTarget(tf::TransformListener& listener)
{
    // 监听从map到base_footprint的变换
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform("map", "body", ros::Time(0), ros::Duration(3.0));
	listener.lookupTransform("map","body",ros::Time(0),transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    // 计算机器人的当前位置
    tf::Vector3 pos(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    double yaw = tf::getYaw(transform.getRotation());
    // 打印接收到的位姿信息
    ROS_INFO("Current position: (%.2f, %.2f, %.2f) , next target index : %d", pos.x(), pos.y(), yaw,point_index);

    // 检查是否接近目标位置
    if(std::abs(pos.x() - target_poses[point_index].x) < threshold.x &&
       std::abs(pos.y() - target_poses[point_index].y) < threshold.y &&
       abs(subAngle(yaw , target_poses[point_index].z)) < threshold.z)
    {
        point_index++;
        if(point_index >= target_poses.size())
            point_index = 0;
        ROS_INFO("Target Changed !!!");
    	count=0;
    }

    if (count-->0)
        return;
    count=10 ;
    
    // 创建目标位姿消息
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "map";
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = target_poses[point_index].x; // 目标X坐标
    target_pose.pose.position.y = target_poses[point_index].y; // 目标Y坐标
    target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target_poses[point_index].z); // 目标角度

    // 发布目标位姿到move_base
    move_base_target_publisher.publish(target_pose);
    ROS_INFO("Target Send!");
}

int main(int argc, char **argv)
{
    // 添加目标点
    target_poses.push_back({0.77,-0.4,-0.36});
    target_poses.push_back({1.59,2.50,0.65});
    target_poses.push_back({4.20,0.13,-1.52});
    target_poses.push_back({3.59,-0.7,-1.52});
    target_poses.push_back({3.59,-4.0,-2.10});
    target_poses.push_back({1.91,-2.83,2.12});
    threshold = {0.5, 0.5, 0.3};

    // 初始化ROS节点
    ros::init(argc, argv, "send_goal_node");
    ros::NodeHandle nh;

    // 创建发布者用于向move_base发送目标位姿
    move_base_target_publisher = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

    // 创建tf监听器
    tf::TransformListener listener;

    // 设置循环率
    ros::Rate loop_rate(2);

    ROS_INFO("Listening for TF transforms...");

    // 开始循环
    while (nh.ok())
    {
        checkAndPublishTarget(listener);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
