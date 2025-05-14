#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LIMO小车里程计数据发布器

该节点读取LIMO小车的左右轮里程计数据，并将其转换为标准ROS里程计消息进行发布。
同时计算并发布小车的位置和姿态信息。

作者: Manus
日期: 2025-04-01
"""

import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from pylimo.limo import LIMO

class LimoOdometryPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('limo_odometry_publisher', anonymous=True)
        
        # 获取ROS参数
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        self.rate = rospy.get_param('~rate', 10.0)  # 发布频率，默认10Hz
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyTHS1')
        self.baud_rate = rospy.get_param('~baud_rate', '460800')
        
        # 初始化LIMO对象
        try:
            self.limo = LIMO(name=self.serial_port, baudrate=self.baud_rate)
            rospy.loginfo("成功连接到LIMO小车")
        except Exception as e:
            rospy.logerr(f"连接LIMO小车失败: {e}")
            return
        
        # 初始化里程计发布器
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        
        # 初始化TF广播器
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 初始化位置和姿态变量
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # 初始化上一次的轮子里程计读数
        self.last_left_wheel_odom = None
        self.last_right_wheel_odom = None
        
        # 小车参数
        self.wheel_radius = 0.0625  # 轮子半径，单位：米
        self.wheel_separation = 0.172  # 轮距，单位：米
        self.ticks_per_meter = 1000.0  # 每米的编码器计数
        
        # 启动里程计发布循环
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.publish_odometry)
        
        rospy.loginfo("LIMO里程计发布器已启动")
    
    def publish_odometry(self, event):
        # 获取当前轮子里程计读数
        left_wheel_odom = self.limo.GetLeftWheelOdeom()
        right_wheel_odom = self.limo.GetRightWheelOdom()
        
        # 如果是第一次读取，初始化上一次的读数
        if self.last_left_wheel_odom is None or self.last_right_wheel_odom is None:
            self.last_left_wheel_odom = left_wheel_odom
            self.last_right_wheel_odom = right_wheel_odom
            return
        
        # 计算轮子转动的距离
        left_wheel_delta = (left_wheel_odom - self.last_left_wheel_odom) / self.ticks_per_meter
        right_wheel_delta = (right_wheel_odom - self.last_right_wheel_odom) / self.ticks_per_meter
        
        # 更新上一次的读数
        self.last_left_wheel_odom = left_wheel_odom
        self.last_right_wheel_odom = right_wheel_odom
        
        # 计算小车的线速度和角速度
        linear_velocity = (right_wheel_delta + left_wheel_delta) / 2.0
        angular_velocity = (right_wheel_delta - left_wheel_delta) / self.wheel_separation
        
        # 获取当前时间
        current_time = rospy.Time.now()
        
        # 计算时间间隔
        dt = 1.0 / self.rate
        
        # 更新位置和姿态
        delta_x = linear_velocity * math.cos(self.th)
        delta_y = linear_velocity * math.sin(self.th)
        delta_th = angular_velocity
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # 创建四元数
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
        # 发布TF变换
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat,
            current_time,
            self.child_frame_id,
            self.frame_id
        )
        
        # 创建并发布里程计消息
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.frame_id
        
        # 设置位置
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))
        
        # 设置速度
        odom.child_frame_id = self.child_frame_id
        odom.twist.twist = Twist(Vector3(linear_velocity, 0, 0), Vector3(0, 0, angular_velocity))
        
        # 发布里程计消息
        self.odom_pub.publish(odom)
        
        # 记录调试信息
        rospy.logdebug(f"位置: ({self.x:.2f}, {self.y:.2f}, {self.th:.2f}), 速度: 线速度={linear_velocity:.2f}, 角速度={angular_velocity:.2f}")

if __name__ == '__main__':
    try:
        odometry_publisher = LimoOdometryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
