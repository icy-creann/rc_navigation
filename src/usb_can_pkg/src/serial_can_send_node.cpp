#include <ros/ros.h>

#include <serial/serial.h>
#include <can_msgs/Frame.h>

#include "can_coder.h"

// 全局变量
serial::Serial ser;
ros::Publisher can_pub;
ros::Subscriber can_sub;

std::string serial_port;
std::string baudrate;
std::string can_downward_topic;

void can_downward_callback(const can_msgs::Frame::ConstPtr &msg)
{
    uint8_t send_data[CAN_LEN_IN_BYTE];
    can_frame send_frame;
    send_frame.id = msg->id;
    send_frame.len = msg->dlc;
    for (int i = 0; i < CAN_DATA_IN_BYTE; i++)
        send_frame.data[i] = msg->data[i];

    encode_can_frame(&send_frame, send_data);

    std::string data_str;
    for (int i = 0; i < CAN_LEN_IN_BYTE; i++)
    {
        char buf[3];
        sprintf(buf, "%02X", send_data[i]);
        data_str += buf;
    }

    try
    {
        size_t len = ser.write(send_data, sizeof(send_data));
        if (len != CAN_LEN_IN_BYTE)
            ROS_INFO("fail to write data, port = %s , data = %s", serial_port.c_str(), data_str.c_str());
        else
            ROS_INFO("send successfully, port = %s , data = %s", serial_port.c_str(), data_str.c_str());
        // 为了防止粘包, 间隔一段时间
        // ros::Duration(0.005).sleep();
        return;
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("IO Exception port : %s", serial_port.c_str());
        return;
    }
    catch (serial::SerialException &e)
    {
        ROS_ERROR("Serial Exception on port : %s", serial_port.c_str());
    }
    catch (serial::PortNotOpenedException &e)
    {
        ROS_ERROR("Port is not opened : %s", serial_port.c_str());
    }
}

bool open_serial()
{
    try
    {
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("IO Exception port : %s", serial_port.c_str());
        return false;
    }
    catch (serial::SerialException &e)
    {
        ROS_ERROR("Serial Exception on port : %s", serial_port.c_str());
        return false;
    }
    catch (serial::PortNotOpenedException &e)
    {
        ROS_ERROR("Port is not opened : %s", serial_port.c_str());
        return false;
    }

    if (ser.isOpen())
    {
        ROS_INFO("Port Opened: %s", serial_port.c_str());
        return true;
    }

    ROS_ERROR("Unable to open port : %s", serial_port.c_str());
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_can_node");
    ros::NodeHandle nh;

    // 获取参数
    nh.getParam("serial_port", serial_port);
    nh.getParam("baud_rate", baudrate);
    nh.getParam("can_downward_topic", can_downward_topic);

    // 打印参数
    if(serial_port.empty())
    {
        ROS_ERROR("serial_port is empty");
        return -1;
    }
    if(baudrate.empty())
    {
        ROS_ERROR("baud_rate is empty");
        return -1;
    }
    if(can_downward_topic.empty())
    {
        ROS_ERROR("can_downward_topic is empty");
        return -1;
    }

    // 发布和订阅
    can_sub = nh.subscribe(can_downward_topic, 1000, can_downward_callback);

    // 初始化串口
    ser.setPort(serial_port);
    ser.setBaudrate(atoi(baudrate.c_str()));
    serial::Timeout timeout_ = serial::Timeout::simpleTimeout(100);
    ser.setTimeout(timeout_);
    open_serial();

    // 帧率上限200
    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        // 尝试发送数据
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}