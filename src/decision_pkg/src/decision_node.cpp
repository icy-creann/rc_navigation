#include "ros/ros.h"

#include "decision.hpp"

#include "interface_pkg/match_info.h"
#include "interface_pkg/target.h"
#include "interface_pkg/position.h"
#include "std_msgs/Int32.h"

using namespace std;


// 初始化类
Decision decision;

Pose current_position; // 机器人的位置和方向，数据来自激光雷达
std::vector<Target> aimed_targets; // 把接收到的目标放进数组中
toNUC_t serialmsg;

// 订阅/发布者
ros::Subscriber  aimed_target_sub;
ros::Subscriber  current_position_sub;
ros::Subscriber  match_info_sub;
ros::Publisher  goal_position_pub;

void update()
{
    // 清除过时的目标
    if (!aimed_targets.empty()) {
        aimed_targets.erase(std::remove_if(aimed_targets.begin(), aimed_targets.end(),
                                           [](const Target& t) { return std::abs(t.t - serialmsg.resT) > 5.0; }),
                            aimed_targets.end());
    }

    // 更新
    decision.update(current_position, aimed_targets, serialmsg);
    
    // 发送
    std_msgs::Int32 targID;
    targID.data = decision.aimTargetId;
    interface_pkg::position p;
    p.x = decision.targetPosition.x;
    p.y = decision.targetPosition.y;
    goal_position_pub.publish(p);

    // 输出日志： (aimed_target_position.x,y) (current_position.x,y) (goal_position.x,y) (match_info.resT)
    float x = 0, y = 0;
    if (!aimed_targets.empty())
    {
        x = aimed_targets[0].x;
        y = aimed_targets[0].y;
    }
    ROS_INFO("aimed_target_position: (%f, %f), current_position: (%f, %f), \
      goal_position: (%f, %f), resT: %d", 
      x, y,
      current_position.x, current_position.y, 
      decision.targetPosition.x, decision.targetPosition.y,
      serialmsg.resT);
}

void Match_info_callback(interface_pkg::match_info p)
{    
    serialmsg.bullet = p.bullet;
    serialmsg.middleState = p.middleState;
    serialmsg.resT = p.resT;
    serialmsg.stage = p.stage;
    serialmsg.HPself_div10 = p.HPself_div10;
    serialmsg.HPselfmax_div10 = p.HPselfmax_div10;
    serialmsg.HPred_div10 = p.HPred_div10;
    serialmsg.HPblue_div10 = p.HPblue_div10;
}

void Current_position_callback(interface_pkg::position p)
{
    current_position.x=p.x;
    current_position.y=p.y;
    current_position.yaw=p.yaw;
    update();
}
void Aimed_target_callback(interface_pkg::target targ)
{
    // 这里 t 中得到的是目标相对机器人自身的坐标
    Target tar;
    tar.x = targ.x;
    tar.y = targ.y;
    tar.z = targ.z;
    tar.t = targ.t;
    tar.type = targ.type;
    tar.id = targ.id;
    aimed_targets.push_back(tar);
    // ROS_ERROR("debug check_point 2");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "decision_node");
  ros::NodeHandle nh;
  
  // 读取参数
  std::string  aimed_target_topic_name, current_position_topic_name, match_info_topic_name, goal_position_topic_name;
  
  nh.getParam("aimed_target_topic", aimed_target_topic_name);
  nh.getParam("current_position_topic", current_position_topic_name);
  nh.getParam("match_info_topic", match_info_topic_name);
  nh.getParam("goal_position_topic", goal_position_topic_name);
  
  // 创建数据收发器
  aimed_target_sub = nh.subscribe(aimed_target_topic_name, 20, Aimed_target_callback);
  current_position_sub = nh.subscribe(current_position_topic_name, 5, Current_position_callback);
  match_info_sub = nh.subscribe(match_info_topic_name, 5, Match_info_callback);

  goal_position_pub = nh.advertise<interface_pkg::position>(goal_position_topic_name, 10);
  
  ros::spin();
  return 0;
}
