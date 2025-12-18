#include <ros/ros.h>
#include <can_msgs/Frame.h>

#include "serial_downward_coder.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_test_node");
    ros::NodeHandle nh;
    
    ros::Publisher can_pub = nh.advertise<can_msgs::Frame>("/can_downward_topic", 10);
    
    ros::Rate loop_rate(100); 
    
    while (ros::ok())
    {
        down_ward_data_raw_float raw_data;
        raw_data.chassis_x = 0.15;
        raw_data.chassis_y = 0;
        raw_data.chassis_yaw = 0;
        raw_data.gimbal_delta_pitch = 0;
        raw_data.gimbal_delta_yaw = 0.2;
        raw_data.gimbal_found = 0;
        raw_data.gimbal_shoot = 0;
        
        down_ward_data data;
        encode_down_ward_data(&raw_data, &data);

        can_msgs::Frame can_msg;
        
        // 设置CAN消息内容
        can_msg.id = 5;
        can_msg.dlc = 8;
        
        memcpy(can_msg.data.begin(), &data, sizeof(down_ward_data));
        
        can_pub.publish(can_msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
