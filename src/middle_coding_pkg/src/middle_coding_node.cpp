#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Twist.h>

#include "interface_pkg/dangle_shoot.h"
#include "interface_pkg/match_info.h"

#include "serial_upward_coder.h"
#include "serial_downward_coder.h"

class MiddleCodingNode
{
private:
    ros::NodeHandle nh_;

    // 订阅者
    ros::Subscriber can_upward_sub;
    ros::Subscriber chassis_control_sub;
    ros::Subscriber gimbal_control_sub;

    // 发布者
    ros::Publisher can_downward_pub;
    ros::Publisher mach_info_pub;

    // 话题名
    std::string can_downward_topic, can_upward_topic, chassis_control_topic, gimbal_control_topic, mach_info_topic, downward_pub_rate;

    // 异步存储下行指令
    geometry_msgs::Twist chassis_control;
    interface_pkg::dangle_shoot gimbal_control;

public:
    ros::Timer timer;
    
    /// @brief 上行指令回调函数
    /// @param msg
    void can_upward_callback(const can_msgs::Frame::ConstPtr &msg)
    {
        uint8_t data[8];
        // 判断帧ID
        if (msg->id != 6)
            return;
        // 读取数据并解码转发
        for (int i = 0; i < 8; i++)
            data[i] = msg->data[i];
        data[0] = 0xAA;


        interface_pkg::match_info match_info;
        decode_upward_serial(data, &match_info);
        mach_info_pub.publish(match_info);
    }

    void chassis_control_callback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        // 保存数据
        chassis_control = *msg;
    }

    void gimbal_control_callback(const interface_pkg::dangle_shoot::ConstPtr &msg)
    {
        // 保存数据
        gimbal_control = *msg;
    }

    void timer_callback(const ros::TimerEvent &event)
    {
        // ROS_INFO("[MiddleCodingNode] Publish Downward Data");
        // 转存
        down_ward_data_raw_float down;
        down.chassis_x = chassis_control.linear.x;
        down.chassis_y = chassis_control.linear.y;
        down.chassis_yaw = chassis_control.angular.z;
        down.gimbal_delta_yaw = gimbal_control.delta_yaw;
        down.gimbal_delta_pitch = gimbal_control.delta_pitch;
        down.gimbal_shoot = gimbal_control.shoot;
        down.gimbal_found = gimbal_control.found;
        // 编码并发送
        uint8_t data[8];
        down_ward_data encoded;
        encode_down_ward_data(&down, &encoded);
        memcpy(data, &encoded, 8);

        can_msgs::Frame frame;
        frame.id = 5;
        frame.dlc = 8;
        frame.is_rtr = false;
        frame.is_extended = false;
        frame.is_error = false;
        for (int i = 0; i < 8; i++)
            frame.data[i] = data[i];
        can_downward_pub.publish(frame);
    }

    MiddleCodingNode()
    {
        // 读取参数
        nh_.getParam("can_downward_topic", can_downward_topic);
        nh_.getParam("can_upward_topic", can_upward_topic);
        nh_.getParam("chassis_control_topic", chassis_control_topic);
        nh_.getParam("gimbal_control_topic", gimbal_control_topic);
        nh_.getParam("mach_info_topic", mach_info_topic);
        nh_.getParam("downward_pub_rate", downward_pub_rate);

        // 初始化订阅者
        can_upward_sub = nh_.subscribe(can_upward_topic, 10, &MiddleCodingNode::can_upward_callback, this);
        chassis_control_sub = nh_.subscribe(chassis_control_topic, 10, &MiddleCodingNode::chassis_control_callback, this);
        gimbal_control_sub = nh_.subscribe(gimbal_control_topic, 10, &MiddleCodingNode::gimbal_control_callback, this);

        // 初始化发布者
        can_downward_pub = nh_.advertise<can_msgs::Frame>(can_downward_topic, 10);
        mach_info_pub = nh_.advertise<interface_pkg::match_info>(mach_info_topic, 10);

        // 初始化计时器
        timer = nh_.createTimer(ros::Duration(1.0 / std::stof(downward_pub_rate)), &MiddleCodingNode::timer_callback, this);

        // 初始化数据
        chassis_control.linear.x = 0;
        chassis_control.linear.y = 0;
        chassis_control.angular.z = 0;
        gimbal_control.delta_yaw = 0;
        gimbal_control.delta_pitch = 0;
        gimbal_control.shoot = 0;
        gimbal_control.found = 0;
    }
};
    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "middle_coding_node");
        MiddleCodingNode node;
        ros::spin();
        return 0;
    }
