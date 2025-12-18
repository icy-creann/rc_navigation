#!/usr/bin/env python

# 导入ros包
import rospy
import tf
import math

from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseActionGoal

# 导入消息包
import interface_pkg.msg as MSG
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

# 仿真管理
class SimulatorManager():
    def __init__(self):
        # 获取参数
        self.aimed_target_topic = rospy.get_param("~aimed_target_topic", "aimed_target_topic")
        self.current_position_topic = rospy.get_param("~current_position_topic", "current_position_topic")
        self.match_info_topic = rospy.get_param("~match_info_topic", "match_info_topic")
        self.goal_position_topic = rospy.get_param("~goal_position_topic", "goal_position_topic")

        # 初始化发布器和订阅器
        self.aimed_target_pub = rospy.Publisher(self.aimed_target_topic, MSG.target, queue_size=10)
        self.current_position_pub = rospy.Publisher(self.current_position_topic, MSG.position, queue_size=10)

        self.match_info_pub = rospy.Publisher(self.match_info_topic, MSG.match_info, queue_size=10)
        
        self.goal_position_pub = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=10)
        self.goal_position_sub = rospy.Subscriber(self.goal_position_topic, MSG.position, self.goal_position_callback)
        
        # 初始化 TF 监听器
        self.tf_listener = tf.TransformListener()
        
        # 初始化服务客户端来检测连通性
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.current_position = None

        # 管理对局信息
        self.match_info = MSG.match_info()
        self.init_match_info()
        
        # 记录事件
        self.simulation_start_time = rospy.get_time()

        # 设置 10Hz 的定时器
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)  # 0.1秒 = 10Hz

        # 订阅地图
        self.map_data = None
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback)

        self.last_goal_position = None

    def init_match_info(self):
        #Example:self.match_info.start = 0
        self.match_info.bullet = 750
        self.match_info.middleState = 1
        self.match_info.resT = 300
        self.match_info.stage = 1
        self.match_info.HPself_div10 = 60
        self.match_info.HPselfmax_div10 = 60
        self.match_info.HPred_div10 = 100
        self.match_info.HPblue_div10 = 100
    
    def update_match_info(self):
        ready_time = 5
        start_time = 0
        simulation_time = rospy.get_time() - self.simulation_start_time
        if simulation_time > ready_time:
            self.match_info.stage = 1
        
        if simulation_time > ready_time + start_time:
            self.match_info.stage = 2
            self.match_info.resT = 300 - int(simulation_time - ready_time - start_time)
            self.match_info.middleState = 1
            # 其它事件可在此处添加
            if int(self.match_info.resT) == 50:
                self.match_info.HPself_div10 = 1
            
            if int(self.match_info.resT) == 40:
                self.match_info.HPself_div10 = 200
            rospy.loginfo("HPself_div10 {%d}",self.match_info.HPself_div10)
        
    def update(self):
        # 更新1帧，频率：10Hz
        # rospy.loginfo("relayer update!")
        self.get_current_position()
        
        target_position = self.get_target_position()
        if target_position != None :
            rospy.loginfo("Nice HZG %f %f",target_position.x, target_position.y)
            #self.pub_target_position(target_position)
        
        self.update_match_info()
        self.match_info_pub.publish(self.match_info)
        
    def pub_target_position(self, target_position):
        """
        发布目标位置
        :param target_position: interface_pkg/target, 目标位置
        """
        if self.current_position == None:
            return
        if self.is_visible(self.current_position, target_position):
            # 直接发布目标位置
            self.aimed_target_pub.publish(target_position)
            #rospy.loginfo("Target position published: (%f, %f)", target_position.x, target_position.y)
        else:
            #rospy.loginfo("Target position not visible: (%f, %f)", target_position.x, target_position.y)
            #rospy.loginfo("target postion: (%f,%f) , current position: (%f,%f)", target_position.x, target_position.y, self.current_position.x, self.current_position.y)
            pass
        
    def get_target_position(self):
        # 获取 enemy_block 的位置
        try:
            # 获取 enemy_block 的索引
            (trans, rot) = self.tf_listener.lookupTransform('map', 'enemy_block', rospy.Time(0))
            target_position = MSG.target()
            target_position.x = trans[0]
            target_position.y = trans[1]
            # target_position.z = math.atan2(2.0 * (rot[3] * rot[2] + rot[0] * rot[1]), 1.0 - 2.0 * (rot[1] * rot[1] + rot[2] * rot[2]))
            target_position.t = self.match_info.resT
            #rospy.loginfo("Nice Ceasar %f %f",target_position.x, target_position.y)
            return target_position
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Failed to get enemy_block position: %s", str(e))
            return None
    
    def get_current_position(self):
        # 获取当前位置
        try:
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            current_position = MSG.position()
            current_position.x = trans[0]
            current_position.y = trans[1]
            current_position.timeStamp = rospy.Time.now()
            self.current_position_pub.publish(current_position)
            self.current_position = current_position
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Failed to get current position: %s", str(e))
    
    def goal_position_callback(self, p):
        # 中继话题给 move_base
        if self.last_goal_position and \
            abs(self.last_goal_position.x - p.x) < 0.05 and \
            abs(self.last_goal_position.y - p.y) < 0.05 and \
            abs(self.last_goal_position.yaw - p.yaw) < 0.05:
            #rospy.loginfo("Received goal position is the same as the last one, not sending again.")
            return

        goal = MoveBaseActionGoal()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.pose.position.x = p.x
        goal.goal.target_pose.pose.position.y = p.y
        q = quaternion_from_euler(0, 0, p.yaw)  # 假设 p.z 是 yaw 角度
        goal.goal.target_pose.pose.orientation.x = q[0]
        goal.goal.target_pose.pose.orientation.y = q[1]
        goal.goal.target_pose.pose.orientation.z = q[2]
        goal.goal.target_pose.pose.orientation.w = q[3]
        
        self.goal_position_pub.publish(goal)
        self.last_goal_position = p

    def timer_callback(self, event):
        # 定时器回调函数，调用 update()
        self.update()

    def map_callback(self, data):
        self.map_data = data

    def is_visible(self, start, end):
        if self.map_data is None:
            rospy.logerr("Map data is None!")
            return False
        
        # 获取地图信息
        map_origin = self.map_data.info.origin.position
        map_resolution = self.map_data.info.resolution
        map_width = self.map_data.info.width
        map_height = self.map_data.info.height

        # 将世界坐标转换为地图坐标
        def world_to_map(x, y):
            mx = int((x - map_origin.x) / map_resolution)
            my = int((y - map_origin.y) / map_resolution)
            return mx, my

        x0, y0 = world_to_map(start.x, start.y)
        x1, y1 = world_to_map(end.x, end.y)
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            index = y0 * map_width + x0
            if  not (self.map_data.data[index] == 0 or self.map_data.data[index] == -1):
                return False
            if x0 == x1 and y0 == y1:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return True

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('simulator_manager')
    
    # 创建 SimulatorManager 实例
    manager = SimulatorManager()
    
    # 保持节点运行
    rospy.spin()