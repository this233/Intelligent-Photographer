# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
#麦科勒姆轮小车移动控制


class Move():
    def __init__(self):
        rospy.init_node('move')
        rospy.loginfo("Move node initial.")
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.info_sub = rospy.Subscriber('/info', Twist, self.info_callback)
        self.ready_pub = rospy.Publisher('/ready', String, queue_size=10)
        self.move_cmd = Twist()
        self.duration = 0
        rospy.spin()

    def info_callback(self, msg):
        #收到/info信息后，将信息转发给/cmd_vel，实现小车移动固定距离
        try:
            # rospy.loginfo("Received /info message: %s", msg)
            rospy.loginfo("Received /info message")

            # 设置线速度和角速度
            # self.move_cmd.linear.x = msg.linear.x /20 # 前后移动速度
            # self.move_cmd.linear.y = msg.linear.y/20 # 左右移动速度
            # self.move_cmd.angular.z = msg.angular.z/20  # 原地旋转速度

            #旋转方向
            self.move_cmd.linear.y = 0.0     
            self.move_cmd.linear.x = 0.0  # 
            self.move_cmd.angular.z = -0.17 if msg.angular.z > 0 else (0.17 if msg.angular.z < 0 else 0)
            # self.move_cmd.angular.z = 0.17  # 原地旋转速度，单位：rad/s 0.17rad/s=10°/s
            self.duration=abs(msg.angular.z)+1 #传过来的单位是角度
            rate = rospy.Rate(10)  # 发布频率，1Hz
            for _ in range(int(self.duration * 10)):  # 持续 duration 秒（1Hz * duration）
                self.cmd_pub.publish(self.move_cmd)
                rate.sleep()
            rospy.loginfo("旋转结束")
            self.stop()
            time.sleep()
            
            #固定速度不同时间
            #x方向
            self.move_cmd.linear.x = 0.1 if msg.linear.x > 0 else (-0.1 if msg.linear.x < 0 else 0)     
            self.move_cmd.linear.y = 0.0  # 左右移动速度，单位：m/s
            self.move_cmd.angular.z = 0.0  # 原地旋转速度，单位：rad/s
            self.duration=abs(msg.linear.x) #传过来的单位是分米
            rate = rospy.Rate(10)  # 发布频率，1Hz
            for _ in range(int(self.duration * 10)):  # 持续 duration 秒（1Hz * duration）
                self.cmd_pub.publish(self.move_cmd)
                rate.sleep()
            rospy.loginfo("x方向移动结束")
            self.stop()
            time.sleep(4)
            
            
            #y方向
            self.move_cmd.linear.y = -0.1 if msg.linear.y > 0 else (0.1 if msg.linear.y < 0 else 0)     
            self.move_cmd.linear.x = 0.0  # 
            self.move_cmd.angular.z = 0.0  # 原地旋转速度，单位：rad/s
            self.duration=abs(msg.linear.y) #传过来的单位是分米
            rate = rospy.Rate(10)  # 发布频率，1Hz
            for _ in range(int(self.duration * 10)):  # 持续 duration 秒（1Hz * duration）
                self.cmd_pub.publish(self.move_cmd)
                rate.sleep()
            rospy.loginfo("y方向移动结束")

            # 停止运动
            self.stop()

        except rospy.ROSInterruptException as e:
            rospy.logerr("Error during callback: %s", e)
            self.stop()

    def stop(self):
        #停止小车运动。
        rospy.loginfo("Stopping the robot.")
        stop_cmd = Twist()  # 所有速度设为0
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_pub.publish(stop_cmd)  # 发布停止命令
        self.ready_pub.publish("ready")

if __name__ == '__main__':
    try:
        Move()  # 创建并运行Move对象
        rospy.loginfo("Move node started.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Move node terminated.")