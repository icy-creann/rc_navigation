#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
//自定义消息接口
#include <interface_pkg/serial_from_stm32.h>
#include <interface_pkg/target.h>
#include <interface_pkg/position.h>
#include <interface_pkg/match_info.h>
#include <interface_pkg/dangle_shoot.h>

#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>

// #define CRUISE_POINT_NUM 6
// #define AROUND_NUM 5
#define PATH_LEN 3

#define STAY_CENTER 2
#define STAY_HOME 2
#define GO_CENTER 1
#define GO_HOME 0

#define THRESHOLD 0.4
#define STOP_TIMES 12

const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

// const float cruise_point[CRUISE_POINT_NUM][3] = {
//     {0.81, -0.61, 0.20},
//     {-4.47, 1.08, -1.25},
//     {-7.78, 2.88, -1.26},
//     {-8.25, 0.02, -1.26},
//     {-4.79, -1.10, -1.25}
// };    // path in the hotel

// const float home_center_path[PATH_LEN][3] = {
//     {-0.01, -0.05, 0.0},
//     {0.00, 3.00, 0.0},
//     {2.0, 4.0, 0.0}
// };
// const float center_point[3] = {2.0, 4.0, 0.0};    
// const float home_point[3] = {-0.01, -0.05, 0.0}; 


//红方点位
const float home_center_path[PATH_LEN][3] = {
    {0.5, 1.0, 0.0},
    {6.0, 0.0, 0.0},
    {5.75, -1.6, 0.0}
};
const float center_point[3] = {5.75, -1.6, 0.0}; 
const float home_point[3] = {0.5, 1.0, 0.0}; 

// //蓝方点位
// const float home_center_path[PATH_LEN][3] = {
//     {11.5, -4.7, 0.0},
//     {6.0, -4.0, 0.0},
//     {5.75, -1.6, 0.0}
// };
// const float center_point[3] = {5.75, -1.6, 0.0}; 
// const float home_point[3] = {11.5, -4.7, 0.0}; 


//
// can_serial   move_base
//     |        ^
//     v       /
// middle_node <---> decision_node 
//     ^
//     |(bridge)
// convert_node <--- 自瞄


// 声明发布者
//发送给decision_node的信息
ros::Publisher serial_msg_pub;//上行串口数据
ros::Publisher my_pos_pub;//（地图系下）自身位置
ros::Publisher aimed_target_pub;//敌人信息
//发送给movebase
ros::Publisher movebase_goal_pub;//要到达的导航目标点
ros::Publisher shoot_pub;

// 声明订阅者
//下面四个订阅转换节点convert_node的消息并合并为interface_pkg::Position类型的敌人信息
ros::Subscriber convert_num_sub;
ros::Subscriber convert_type_sub;
//ros::Subscriber convert_dis_to_img_sub;
ros::Subscriber convert_pose_sub;
//订阅antispinner在convert_node中转换来的DangleShoot消息
ros::Subscriber convert_delta_pitch_sub;
ros::Subscriber convert_delta_yaw_sub;
ros::Subscriber convert_shoot_sub;

ros::Subscriber goal_position_sub;//接收从decision_node传来的导航位置消息goal_position,并将导航信息goal发给move_base
ros::Subscriber opp_pos_sub;//从Aim自瞄节点接收的敌人位置消息
ros::Subscriber can_serial_sub;//从can_serial_node接收串口信息，并发送给decision_node
                        

// 声明其他全局变量
double start_timepoint;//记录开始时间点
double passed_time;//记录已经过去的时间
interface_pkg::position my_pos; //记录自身位置信息



int flag_tar[3] = {0};//用于判断是否收齐来自convert_node的一组信息，收齐后，发送给decision_node
interface_pkg::target tar;//用于在四个convert_函数中存储敌人信息
int flag_shoot[3] = {0};//用于判断是否收齐来自convert_node的dangle_shoot信息
interface_pkg::dangle_shoot DangleShoot;//用于存储来自convert_node的dangle_shoot信息

void update();
void goal_callback(const interface_pkg::position::ConstPtr& goal_pos);
void serial_callback(const interface_pkg::match_info::ConstPtr& msg);
void convert_num(const std_msgs::String::ConstPtr& number);
void convert_type(const std_msgs::String::ConstPtr& type);
// void convert_dis_to_img(const std_msgs::Float32::ConstPtr& type);
void convert_pose(const geometry_msgs::Pose::ConstPtr& pose);
void convert_delta_pitch(const std_msgs::Float32::ConstPtr& delta_pitch);
void convert_delta_yaw(const std_msgs::Float32::ConstPtr& delta_yaw);
void convert_shoot(const std_msgs::Bool::ConstPtr& shoot);


int main(int argc, char **argv)
{   
    ROS_INFO("middle_node");
    ros::init(argc, argv, "middle_node");//初始化middle_node节点

    ros::NodeHandle nh;//创建一个节点句柄nh

    //定义发布者，并授予其advertise到指定topic的权限
    serial_msg_pub = nh.advertise<interface_pkg::serial_from_stm32>("/serialmsg_Topic", 10);
    my_pos_pub = nh.advertise<geometry_msgs::Point>("/current_position_topic", 10);
    aimed_target_pub = nh.advertise<interface_pkg::target>("/aimed_target_topic", 10);
    movebase_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);//向move_base发送目标位姿
    shoot_pub = nh.advertise<interface_pkg::dangle_shoot>("/dangle_shoot", 10);
    
    //定义订阅者，并订阅指定的topic，当有消息发布到该topic时，调用回调函数
    goal_position_sub = nh.subscribe<interface_pkg::position>("/goal_position_topic", 10, goal_callback);
    can_serial_sub = nh.subscribe<interface_pkg::match_info>("/match_info_topic", 10, serial_callback);
    convert_num_sub = nh.subscribe<std_msgs::String>("/convert/number", 10, convert_num);
    convert_type_sub = nh.subscribe<std_msgs::String>("/convert/type", 10, convert_type);
    // convert_dis_to_img_sub = nh.subscribe<std_msgs::Float32>("/convert/dis_to_img", 10, convert_dis_to_img);
    convert_pose_sub = nh.subscribe<geometry_msgs::Pose>("/convert/pose", 10, convert_pose);
    convert_delta_pitch_sub = nh.subscribe<std_msgs::Float32>("/convert/delta_pitch", 10, convert_delta_pitch);
    convert_delta_yaw_sub = nh.subscribe<std_msgs::Float32>("/convert/delta_yaw", 10, convert_delta_yaw);
    convert_shoot_sub = nh.subscribe<std_msgs::Bool>("/convert/shoot", 10, convert_shoot);

    start_timepoint = ros::Time::now().sec;//记录开始时间，用于作差计算对局时长

    ros::Rate loop_rate(50);//设置循环频率
    while(ros::ok())//创建循环
    {
        update();//更新信息,并发送自身位置消息position给decision_node
        ros::spinOnce();//处理回调
        loop_rate.sleep();//按照设定的频率等待
    }
    return 0;
}

//信息更新,并发送自身位置消息给decision_node
void update(){
    passed_time = ros::Time::now().sec - start_timepoint;

// //单独测试用（因为单独测试时等待不到tf变换）
//     interface_pkg::position my_pos;
//     my_pos_pub.publish(my_pos);

    // 设置目标坐标系和源坐标系的名称
    std::string target_frame = "body";// 实时扫描得出的坐标系（目标坐标系）
    std::string source_frame = "map";// 地图坐标系（源坐标系）

// 通过tf获取从地图坐标系到自身位置坐标系的转换transform
    //创建一个tf::TransformListener对象listener，用于监听坐标变换
    tf::TransformListener listener;
//添加异常处理提高代码的健壮性
    try {
    //lookupTransform下参数含义:
        // source_frame: the frame where the data originated.
        // target_frame: the frame to which data should be transformed.
        // target_frame是/turtle2，source_frame是/turtle1，是从source的源数据，转换到target的坐标系变换。
        //listener.waitForTransform("/turtle2目标坐标系", "/turtle1源坐标系", ros::Time(0), ros::Duration(3.0))

        //等待目标坐标系（target_frame）到源坐标系（source_frame）之间的变换关系在tf树中生效，最多等待3秒
        listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));
        tf::StampedTransform transform;
        //获取从map到body的变换，并将结果存储在transform中
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);

    //通过坐标系变换获取自身位置并发布
        my_pos.x = transform.getOrigin().x();
        my_pos.y = transform.getOrigin().y();
        my_pos.yaw = tf::getYaw(transform.getRotation());
        my_pos_pub.publish(my_pos);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("Transform error: %s", ex.what());
    }
}

  
// 接收convert_node的敌人number消息并整合到tar中
void convert_num(const std_msgs::String::ConstPtr& number)
{
    int num = std::stoi(number->data); 
    if (num < 0 || num > 6)// 异常
    {
        tar.type = 2;
        ROS_ERROR("ERROR: armor.number=%d, not in range of 0-6", num);
    }else if (num >=0 && num <= 2)//0，1，2为大装甲板
    {
        tar.type = 1;
    }else{// 3,4,5,6为小装甲板
        tar.type = 0;
    }
    
    tar.id = num;
    flag_tar[0] = 1;
    //同一组的三个消息整合完毕时才发送一次
    if(flag_tar[1] && flag_tar[2] && flag_tar[0]){
      tar.timeStamp = ros::Time::now();
      tar.t = passed_time;
      aimed_target_pub.publish(tar);
      flag_tar[0] = 0, flag_tar[1] = 0, flag_tar[2] = 0;
      ROS_INFO("Published: number = %d, type = %s ", tar.id, ARMOR_TYPE_STR[num].c_str());
    }
}

// 接收convert_node的敌人type消息并整合到tar中
void convert_type(const std_msgs::String::ConstPtr& type)
{
    // tar.type = std::stoi(type->data); //与number传递的消息重合了
    flag_tar[1] = 1;
    if(flag_tar[2] && flag_tar[0] && flag_tar[1]){
      tar.timeStamp = ros::Time::now();
      tar.t = passed_time;
      aimed_target_pub.publish(tar);
      flag_tar[0] = 0, flag_tar[1] = 0, flag_tar[2]= 0;
      ROS_INFO("Published: number = %d, type = %d ", tar.id, tar.type);
    }
}

//dis_to_img信息没有用到
// void convert_dis_to_img(const std_msgs::Float32::ConstPtr& type)
// {
//     tar.type = type->data;
//     flag_tar[2] = 1;
//     if(flag_tar[1] && flag_tar[2] && flag_tar[3] && flag_tar[0]){
//       tar.timeStamp = ros::Time::now();
//       tar.t = passed_time;
//       aimed_target_pub.publish(tar);
//       flag_tar[0] = 0, flag_tar[1] = 0, flag_tar[2]= 0, flag_tar[3] = 0;
//     }
// }

// 接收convert_node的敌人pose消息并整合到tar中
void convert_pose(const geometry_msgs::Pose::ConstPtr& pose)
{
    tar.x = pose->position.x;
    tar.y = pose->position.y;
    tar.z = pose->position.z;
    flag_tar[2] = 1;
    if(flag_tar[1] && flag_tar[0] && flag_tar[2]){
      tar.timeStamp = ros::Time::now();
      tar.t = passed_time;
      aimed_target_pub.publish(tar);

      flag_tar[0] = 0, flag_tar[1] = 0, flag_tar[2]= 0;
      ROS_INFO("Published: number = %d, type = %d ", tar.id, tar.type);
    }
}

void convert_delta_pitch(const std_msgs::Float32::ConstPtr& delta_pitch)
{
    DangleShoot.delta_pitch = delta_pitch->data;
    flag_shoot[0] = 1;
    if(flag_shoot[1] && flag_shoot[2] && flag_shoot[0]){
      shoot_pub.publish(DangleShoot);
    }
}

void convert_delta_yaw(const std_msgs::Float32::ConstPtr& delta_yaw)
{
    DangleShoot.delta_yaw = delta_yaw->data;
    flag_shoot[1] = 1;
    if(flag_shoot[0] && flag_shoot[2] && flag_shoot[1]){
      shoot_pub.publish(DangleShoot);
    }
}

void convert_shoot(const std_msgs::Bool::ConstPtr& shoot)
{
    DangleShoot.shoot = shoot->data;
    flag_shoot[2] = 1;
    if(flag_shoot[0] && flag_shoot[1] && flag_shoot[2]){
      shoot_pub.publish(DangleShoot);
    }
}

void goal_callback(const interface_pkg::position::ConstPtr& goal_pos)
{
    // geometry_msgs::PoseStamped goal;//定义一个导航目标goal

    // goal.header.frame_id = "map";// 认为导航目标goal是在地图坐标系下描述的
    // goal.header.stamp = ros::Time::now();
    // goal.pose.position.x = goal_pos->x;
    // goal.pose.position.y = goal_pos->y;
    // //四元数q记录yaw转化来的旋转信息
    // geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(goal_pos->yaw);
    // goal.pose.orientation = q;
    // movebase_goal_pub.publish(goal);
    // // ac.waitForServer();
    // // ac.sendGoal(goal);
}


void go_center(int *status) {
    static int center_path = 0;
    static int loop_center = 0;
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    float targ_x = home_center_path[center_path][0];
    float targ_y = home_center_path[center_path][1];
    float targ_yaw = 0.0;

    goal.pose.position.x = targ_x;
    goal.pose.position.y = targ_y;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(targ_yaw);
    for (int i = 0; i < 10; i++) {
        movebase_goal_pub.publish(goal);
    }

    loop_center ++;
    //判断是否到达，
    double dx = targ_x - my_pos.x;
    double dy = targ_y - my_pos.y;
    double distance_to_goal = std::sqrt(dx * dx + dy * dy);

    
    if (distance_to_goal < THRESHOLD || loop_center >= STOP_TIMES) { // 判断是否到达目标点，阈值为0.4米
        if (center_path >= PATH_LEN-1) { // 如果已经到达了中心点，则保持不动
            center_path = 0;
            *status = STAY_CENTER;
            return;
        }
        center_path += 1;
    }

    ROS_INFO("[middle_decision]: Go to center[%f, %f] dx_dy:[%f, %f]", targ_x, targ_y, dx, dy);
}


void go_home(int *status){
    static int home_path = PATH_LEN-1;
    static int loop_home = 0;
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    float targ_x = home_center_path[home_path][0];
    float targ_y = home_center_path[home_path][1];
    float targ_yaw = 0.0;

    goal.pose.position.x = targ_x;
    goal.pose.position.y = targ_y;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(targ_yaw);
    for (int i = 0; i < 10; i++) {
        movebase_goal_pub.publish(goal);
    }

    //判断是否到达，
    double dx = targ_x - my_pos.x;
    double dy = targ_y - my_pos.y;
    double distance_to_goal = std::sqrt(dx * dx + dy * dy);

    
    if (distance_to_goal < THRESHOLD || loop_home >= STOP_TIMES) { // 判断是否到达目标点，阈值为0.4米
        if (home_path <= 0) { // 如果已经到达了泉水，则保持不动
            home_path = PATH_LEN;
            *status = STAY_HOME;  //停留，直到回够血
            return;
        }
        home_path -= 1;
    }

    ROS_INFO("[middle_decision]: Go to base[%f, %f] dx_dy:[%f, %f]", targ_x, targ_y, dx, dy);
}

// 坚守中心点
void keep_center() {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    goal.pose.position.x = center_point[0];
    goal.pose.position.y = center_point[1];
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    for (int i = 0; i < 10; i++) {
        movebase_goal_pub.publish(goal);
    }
}

void keep_home() {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    goal.pose.position.x = home_point[0];
    goal.pose.position.y = home_point[1];
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    for (int i = 0; i < 10; i++) {
        movebase_goal_pub.publish(goal);
    }
}

//发布上行串口数据
void serial_callback(const interface_pkg::match_info::ConstPtr& msg)
{
    static int status = GO_CENTER;//开局先抢占中心增益点
    // interface_pkg::match_info serialmsg;
    serial_msg_pub.publish(*msg);
    
//调试时注释
    if (msg->stage != '4') //非对战中，不进行行动 // 0 未开始；1 准备；2 自检；3 倒计时； 4 对战中； 5 比赛结束
    {
        ROS_INFO("[middle_decision]: Not in battle, no action.");
        return;
    }

    if (status == GO_CENTER) 
    {
        go_center(&status);
        return;
    }
    
    int HPself = msg->HPself_div10 * 10; //计算自身血量
    ROS_INFO("HP:%d", HPself);
    if (HPself < 220 || status == GO_HOME) 
    {
        //回泉水
        status = GO_HOME;
        go_home(&status);
    }else if((status == GO_HOME || status == GO_CENTER) && HPself > 380){ //此前回/待在泉水，且此时血量大于380，重回中心点
        status = GO_CENTER;
        go_center(&status);
    }else if(status == STAY_CENTER){
        keep_center();// 坚守中心点
    }else if(status == STAY_HOME){
        keep_home();// 待在泉水回血
    }
    
}



// // 定点巡航的逻辑
// void cruise(interface_pkg::position my_pos, int status) {
//     static int center_path = 0;
//     static int point_num = 0;
//     geometry_msgs::PoseStamped goal;
//     goal.header.frame_id = "map";
//     goal.header.stamp = ros::Time::now();


//     if (status == CRUISE) { // 如果之前已经在定点巡航，按顺序巡航即可
//         ;
//     }else{ // 之前没有定点巡航，现在重新巡航时找一个最近的点开始
//         double min_distance = std::numeric_limits<double>::max();
//         int closest_point = 0;

//         for (int i = 0; i < CRUISE_POINT_NUM; ++i) {
//             double dx = cruise_point[i][0] - my_pos.x;
//             double dy = cruise_point[i][1] - my_pos.y;
//             double distance = std::sqrt(dx * dx + dy * dy);

//             if (distance < min_distance) {
//                 min_distance = distance;
//                 closest_point = i;
//             }
//         }
//         point_num = closest_point;
//     }
    
//     goal.pose.position.x = cruise_point[point_num][0];
//     goal.pose.position.y = cruise_point[point_num][1];
//     goal.pose.orientation = tf::createQuaternionMsgFromYaw(cruise_point[point_num][2]);
   
//     // 若到达之前设定的巡航点，准备下一个巡航点
//     double dx = cruise_point[point_num][0] - my_pos.x;
//     double dy = cruise_point[point_num][1] - my_pos.y;
//     double distance_to_goal = std::sqrt(dx * dx + dy * dy);

//     if (distance_to_goal < THRESHOLD) { // 判断是否到达目标点，阈值为0.4米
//         point_num = (point_num + 1) % CRUISE_POINT_NUM;
//     }
    
//     movebase_goal_pub.publish(goal);
//     ROS_INFO("[middle_decision]: Cruising to:[%f, %f]  dx_dy:[%f, %f]", goal.pose.position.x, goal.pose.position.y, dx, dy);
// }