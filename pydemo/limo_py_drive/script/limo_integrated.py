#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LIMO小车集成控制程序

该程序集成了LIMO小车的所有控制功能，包括：
1. 里程计数据发布
2. 状态信息发布
3. 麦轮控制模式切换
4. IMU数据发布
5. 错误检测与速度控制
6. 键盘和鼠标控制选项

通过集成所有功能到一个Python文件中，解决了多个程序同时打开串口导致冲突的问题。

作者: Manus
日期: 2025-04-02
"""

import rospy
import math
import numpy as np
import threading
import sys
import termios
import tty
import tkinter as tk
import tf
from std_msgs.msg import Bool, Float32, UInt8, UInt16, String
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from pylimo.limo import LIMO
from pylimo import limomsg

class LimoIntegratedController:
    """
    LIMO小车集成控制器类
    
    该类集成了LIMO小车的所有控制功能，共享同一个串口连接。
    """
    
    def __init__(self):
        """
        初始化LIMO小车集成控制器
        
        参数:
            无
            
        返回:
            无
        """
        # 初始化ROS节点
        rospy.init_node('limo_integrated_controller', anonymous=True)
        
        # 获取通用ROS参数
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyTHS1')
        self.baud_rate = rospy.get_param('~baud_rate', '460800')
        
        # 获取功能启用参数
        self.enable_odometry = rospy.get_param('~enable_odometry', True)
        self.enable_status = rospy.get_param('~enable_status', True)
        self.enable_mode_control = rospy.get_param('~enable_mode_control', True)
        self.enable_imu = rospy.get_param('~enable_imu', True)
        self.enable_error_detection = rospy.get_param('~enable_error_detection', True)
        self.control_type = rospy.get_param('~control_type', 'keyboard')  # 'keyboard' 或 'mouse'
        
        # 获取里程计参数
        if self.enable_odometry:
            self.odom_rate = rospy.get_param('~odom_rate', 10.0)
            self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
            self.odom_child_frame_id = rospy.get_param('~odom_child_frame_id', 'base_link')
        
        # 获取状态信息参数
        if self.enable_status:
            self.status_rate = rospy.get_param('~status_rate', 1.0)
        
        # 获取模式控制参数
        if self.enable_mode_control:
            self.default_mode = rospy.get_param('~default_mode', 0)  # 默认差速模式
            self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)
            self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.0)
            self.max_lateral_speed = rospy.get_param('~max_lateral_speed', 0.8)
        
        # 获取IMU参数
        if self.enable_imu:
            self.imu_rate = rospy.get_param('~imu_rate', 50.0)
            self.imu_frame_id = rospy.get_param('~imu_frame_id', 'imu_link')
            self.use_filter = rospy.get_param('~use_filter', True)
            self.filter_size = rospy.get_param('~filter_size', 5)
        
        # 获取错误检测参数
        if self.enable_error_detection:
            self.check_rate = rospy.get_param('~check_rate', 10.0)
            self.auto_recovery = rospy.get_param('~auto_recovery', True)
            self.normal_max_speed = rospy.get_param('~normal_max_speed', 1.0)
            self.warning_max_speed = rospy.get_param('~warning_max_speed', 0.5)
            self.critical_max_speed = rospy.get_param('~critical_max_speed', 0.2)
            self.battery_warning_threshold = rospy.get_param('~battery_warning_threshold', 11.0)
            self.battery_critical_threshold = rospy.get_param('~battery_critical_threshold', 10.0)
        
        # 初始化LIMO对象（共享同一个串口连接）
        try:
            self.limo = LIMO(name=self.serial_port, baudrate=self.baud_rate)
            rospy.loginfo("成功连接到LIMO小车")
            
            # 启用命令控制模式
            # if motion_mode == self.DIFF_MODE and motion_mode == self.ACKERMANN_MODE:
                
            #     self.limo.EnableCommand()
            #     rospy.loginfo("已启用命令控制模式")
            # else:
            #     self.limo.EnableMec()
            #     rospy.loginfo("已启mec控制模式")                

            
        except Exception as e:
            rospy.logerr(f"连接LIMO小车失败: {e}")
            return
        
        # 初始化里程计变量
        if self.enable_odometry:
            self.x = 0.0
            self.y = 0.0
            self.th = 0.0
            self.last_left_wheel_odom = None
            self.last_right_wheel_odom = None
            self.wheel_radius = 0.0625  # 轮子半径，单位：米
            self.wheel_separation = 0.172  # 轮距，单位：米
            self.ticks_per_meter = 1000.0  # 每米的编码器计数
            
            # 初始化里程计发布器
            self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
            
            # 初始化TF广播器
            self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 初始化状态信息变量
        if self.enable_status:
            # 错误代码映射表
            self.error_code_map = {
                0x0001: "电池电量低",
                0x0002: "电池电量严重不足",
                0x0004: "遥控器连接丢失",
                0x0008: "电机1驱动错误",
                0x0010: "电机2驱动错误",
                0x0020: "电机3驱动错误",
                0x0040: "电机4驱动错误",
                0x0100: "驱动状态错误"
            }
            
            # 初始化状态信息发布器
            self.battery_pub = rospy.Publisher('limo_status/battery', Float32, queue_size=10)
            self.control_mode_pub = rospy.Publisher('limo_status/control_mode', UInt8, queue_size=10)
            self.vehicle_state_pub = rospy.Publisher('limo_status/vehicle_state', UInt8, queue_size=10)
            self.motion_mode_pub = rospy.Publisher('limo_status/motion_mode', UInt8, queue_size=10)
            self.error_code_pub = rospy.Publisher('limo_status/error_code', UInt16, queue_size=10)
            self.error_desc_pub = rospy.Publisher('limo_status/error_description', String, queue_size=10)
            
            # 记录上一次的错误代码，用于检测错误状态变化
            self.last_error_code = 0
        
        # 初始化模式控制变量
        if self.enable_mode_control:
            # 运动模式常量
            self.DIFF_MODE = 0    # 差速模式
            self.ACKERMANN_MODE = 1  # 阿克曼模式
            self.MECANUM_MODE = 2  # 麦轮模式
            
            # 设置默认模式
            self.current_mode = self.default_mode
            limomsg.SetMotionMode(self.current_mode)
            
            # 创建模式切换服务
            self.diff_mode_service = rospy.Service('set_diff_mode', SetBool, self.set_diff_mode_callback)
            self.ackermann_mode_service = rospy.Service('set_ackermann_mode', SetBool, self.set_ackermann_mode_callback)
            self.mecanum_mode_service = rospy.Service('set_mecanum_mode', SetBool, self.set_mecanum_mode_callback)
            
            # 创建模式发布器
            self.mode_pub = rospy.Publisher('limo_mode', UInt8, queue_size=10)
            
            # 创建速度命令订阅器
            self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        # 初始化IMU变量
        if self.enable_imu:
            # 初始化IMU发布器
            self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=50)
            
            # 初始化滤波器缓冲区
            if self.use_filter:
                self.accel_buffer = []
                self.gyro_buffer = []
                self.euler_buffer = []
                for _ in range(self.filter_size):
                    self.accel_buffer.append((0.0, 0.0, 0.0))
                    self.gyro_buffer.append((0.0, 0.0, 0.0))
                    self.euler_buffer.append((0.0, 0.0, 0.0))
            
            # 协方差矩阵
            self.orientation_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
            self.angular_velocity_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
            self.linear_acceleration_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        
        # 初始化错误检测变量
        if self.enable_error_detection:
            # 初始化状态变量
            self.error_state = 0  # 0: 正常, 1: 警告, 2: 严重警告, 3: 错误
            self.emergency_stop = False  # 紧急停止标志
            self.last_cmd_vel = Twist()  # 上一次的速度命令
            
            # 创建锁，用于线程同步
            self.lock = threading.Lock()
            
            # 创建发布器
            self.error_state_pub = rospy.Publisher('limo_error_state', Float32, queue_size=10)
            self.emergency_stop_pub = rospy.Publisher('limo_emergency_stop', Bool, queue_size=10)
            
            # 创建服务
            self.emergency_stop_srv = rospy.Service('emergency_stop', Trigger, self.emergency_stop_callback)
            self.emergency_recovery_srv = rospy.Service('emergency_recovery', Trigger, self.emergency_recovery_callback)
        
        # 初始化控制变量
        if self.control_type == 'keyboard':
            # 键盘控制参数
            self.linear_speed_step = rospy.get_param('~linear_speed_step', 0.1)  # 线速度步进值
            self.angular_speed_step = rospy.get_param('~angular_speed_step', 0.1)  # 角速度步进值
            self.lateral_speed_step = rospy.get_param('~lateral_speed_step', 0.1)  # 横向速度步进值
            
            # 初始化速度值
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0
            
            # 创建键盘监听线程
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
            self.keyboard_thread.daemon = True
            self.keyboard_thread.start()
            
            # 显示控制说明
            self.print_keyboard_instructions()
            
        elif self.control_type == 'mouse':
            # 初始化速度值
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0
            
            # 创建GUI线程
            self.gui_thread = threading.Thread(target=self.create_gui)
            self.gui_thread.daemon = True
            self.gui_thread.start()
        
        # 启动定时器
        if self.enable_odometry:
            self.odom_timer = rospy.Timer(rospy.Duration(1.0/self.odom_rate), self.publish_odometry)
        
        if self.enable_status:
            self.status_timer = rospy.Timer(rospy.Duration(1.0/self.status_rate), self.publish_status)
        
        if self.enable_mode_control:
            self.mode_timer = rospy.Timer(rospy.Duration(1.0), self.publish_mode)
        
        if self.enable_imu:
            self.imu_timer = rospy.Timer(rospy.Duration(1.0/self.imu_rate), self.publish_imu)
        
        if self.enable_error_detection:
            self.error_check_timer = rospy.Timer(rospy.Duration(1.0/self.check_rate), self.check_errors)
            self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_error_status)
        
        if self.control_type in ['keyboard', 'mouse']:
            self.cmd_timer = rospy.Timer(rospy.Duration(0.1), self.publish_cmd_vel)
        
        rospy.loginfo("LIMO小车集成控制器已启动")
    
    # ===== 里程计功能 =====
    
    def publish_odometry(self, event):
        """
        发布里程计数据
        
        参数:
            event: 定时器事件
            
        返回:
            无
        """
        if not self.enable_odometry:
            return
        
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
        dt = 1.0 / self.odom_rate
        
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
            self.odom_child_frame_id,
            self.odom_frame_id
        )
        
        # 创建并发布里程计消息
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame_id
        
        # 设置位置
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))
        
        # 设置速度
        odom.child_frame_id = self.odom_child_frame_id
        odom.twist.twist = Twist(Vector3(linear_velocity, 0, 0), Vector3(0, 0, angular_velocity))
        
        # 发布里程计消息
        self.odom_pub.publish(odom)
        
        # 记录调试信息
        rospy.logdebug(f"位置: ({self.x:.2f}, {self.y:.2f}, {self.th:.2f}), 速度: 线速度={linear_velocity:.2f}, 角速度={angular_velocity:.2f}")
    
    # ===== 状态信息功能 =====
    
    def publish_status(self, event):
        """
        发布状态信息
        
        参数:
            event: 定时器事件
            
        返回:
            无
        """
        if not self.enable_status:
            return
        
        # 获取电池电压
        battery_voltage = self.limo.GetBatteryVoltage()
        self.battery_pub.publish(Float32(battery_voltage))
        
        # 获取控制模式
        control_mode = self.limo.GetControlMode()
        self.control_mode_pub.publish(UInt8(control_mode))
        
        # 获取车辆状态
        vehicle_state = limomsg.GetVehicleState()
        self.vehicle_state_pub.publish(UInt8(vehicle_state))
        
        # 获取运动模式
        motion_mode = limomsg.GetMotionMode()
        self.motion_mode_pub.publish(UInt8(motion_mode))
        
        # 获取错误代码
        error_code = self.limo.GetErrorCode()
        self.error_code_pub.publish(UInt16(error_code))
        
        # 解析错误代码并生成错误描述
        error_descriptions = []
        for bit, description in self.error_code_map.items():
            if error_code & bit:
                error_descriptions.append(description)
        
        # 如果没有错误，添加一个"正常"描述
        if not error_descriptions:
            error_descriptions.append("正常")
        
        # 发布错误描述
        self.error_desc_pub.publish(String(", ".join(error_descriptions)))
        
        # 检测错误状态变化并记录日志
        if error_code != self.last_error_code:
            if error_code > self.last_error_code:
                # 新增错误
                for bit, description in self.error_code_map.items():
                    if (error_code & bit) and not (self.last_error_code & bit):
                        rospy.logwarn(f"LIMO错误: {description}")
            else:
                # 错误恢复
                for bit, description in self.error_code_map.items():
                    if (self.last_error_code & bit) and not (error_code & bit):
                        rospy.loginfo(f"LIMO错误已恢复: {description}")
            
            # 更新上一次的错误代码
            self.last_error_code = error_code
        
        # 电池电量警告
        if battery_voltage < 10.0:
            rospy.logwarn(f"电池电量严重不足: {battery_voltage:.2f}V")
        elif battery_voltage < 11.0:
            rospy.logwarn(f"电池电量低: {battery_voltage:.2f}V")
        
        # 记录调试信息
        rospy.logdebug(f"电池电压: {battery_voltage:.2f}V, 控制模式: {control_mode}, " +
                      f"车辆状态: {vehicle_state}, 运动模式: {motion_mode}, " +
                      f"错误代码: {error_code}")
    
    # ===== 模式控制功能 =====
    
    def get_mode_name(self, mode):
        """
        获取模式名称
        
        参数:
            mode: 模式编号
            
        返回:
            模式名称字符串
        """
        if mode == self.DIFF_MODE:
            return "差速模式"
        elif mode == self.ACKERMANN_MODE:
            return "阿克曼模式"
        elif mode == self.MECANUM_MODE:
            return "麦轮模式"
        else:
            return "未知模式"
    
    def set_motion_mode(self, mode):
        """
        设置运动模式
        
        参数:
            mode: 模式编号
            
        返回:
            无
        """
        if not self.enable_mode_control:
            return
        
        # 设置运动模式
        limomsg.SetMotionMode(mode)
        self.current_mode = mode
        rospy.loginfo(f"已切换到{self.get_mode_name(mode)}")
        
        # 切换模式后停止小车
        self.limo.SetMotionCommand(0, 0, 0, 0)
    
    def set_diff_mode_callback(self, req):
        """
        差速模式服务回调
        
        参数:
            req: 服务请求
            
        返回:
            服务响应
        """
        if req.data:
            self.set_motion_mode(self.DIFF_MODE)
            return SetBoolResponse(True, f"已切换到{self.get_mode_name(self.DIFF_MODE)}")
        return SetBoolResponse(False, "操作被取消")
    
    def set_ackermann_mode_callback(self, req):
        """
        阿克曼模式服务回调
        
        参数:
            req: 服务请求
            
        返回:
            服务响应
        """
        if req.data:
            self.set_motion_mode(self.ACKERMANN_MODE)
            return SetBoolResponse(True, f"已切换到{self.get_mode_name(self.ACKERMANN_MODE)}")
        return SetBoolResponse(False, "操作被取消")
    
    def set_mecanum_mode_callback(self, req):
        """
        麦轮模式服务回调
        
        参数:
            req: 服务请求
            
        返回:
            服务响应
        """
        if req.data:
            self.set_motion_mode(self.MECANUM_MODE)
            return SetBoolResponse(True, f"已切换到{self.get_mode_name(self.MECANUM_MODE)}")
        return SetBoolResponse(False, "操作被取消")
    
    def publish_mode(self, event):
        """
        发布当前模式
        
        参数:
            event: 定时器事件
            
        返回:
            无
        """
        if not self.enable_mode_control:
            return
        
        self.mode_pub.publish(UInt8(self.current_mode))
    
    def cmd_vel_callback(self, msg):
        """
        速度命令回调
        
        参数:
            msg: Twist消息
            
        返回:
            无
        """
        if not self.enable_mode_control:
            return
        
        # 如果使用键盘或鼠标控制，则忽略外部速度命令
        if self.control_type in ['keyboard', 'mouse']:
            return
        
        # 如果错误检测功能已启用且紧急停止已激活，则忽略速度命令
        if self.enable_error_detection and self.emergency_stop:
            rospy.logdebug("紧急停止已激活，忽略速度命令")
            return
        
        # 获取速度命令
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        
        # 如果错误检测功能已启用，则应用速度限制
        if self.enable_error_detection:
            # 根据错误状态调整速度
            if self.error_state == 2:  # 严重警告
                max_speed = self.critical_max_speed
                rospy.logdebug(f"严重警告状态，限制速度为 {max_speed} m/s")
            elif self.error_state == 1:  # 警告
                max_speed = self.warning_max_speed
                rospy.logdebug(f"警告状态，限制速度为 {max_speed} m/s")
            else:  # 正常
                max_speed = self.normal_max_speed
            
            # 限制线速度
            linear_x_scale = abs(linear_x) / max(abs(linear_x), 0.001)
            linear_y_scale = abs(linear_y) / max(abs(linear_y), 0.001)
            
            if linear_x_scale > 1.0 or linear_y_scale > 1.0:
                scale = min(1.0 / linear_x_scale, 1.0 / linear_y_scale)
                linear_x *= scale * max_speed / self.normal_max_speed
                linear_y *= scale * max_speed / self.normal_max_speed
                angular_z *= scale  # 同比例缩放角速度
        
        # 发送速度命令
        self.send_motion_command(linear_x, linear_y, angular_z)
    
    def send_motion_command(self, linear_x, linear_y, angular_z):
        """
        发送运动命令
        
        参数:
            linear_x: X轴线速度
            linear_y: Y轴线速度
            angular_z: Z轴角速度
            
        返回:
            无
        """
        if not self.enable_mode_control:
            return
        
        # 获取当前运动模式
        motion_mode = self.current_mode
        
        if motion_mode == self.DIFF_MODE:  # 差速模式
            self.limo.EnableCommand()
            rospy.loginfo("已启用命令控制模式")
            self.limo.SetMotionCommand(linear_x, angular_z, 0, 0)
            rospy.logdebug(f"差速模式控制: 线速度={linear_x:.2f}, 角速度={angular_z:.2f}")
            
        elif motion_mode == self.ACKERMANN_MODE:  # 阿克曼模式
            # 计算转向角度
            wheelbase = 0.2  # 轴距
            if abs(linear_x) < 0.1 or abs(angular_z) < 0.001:
                # 速度太小或直线行驶
                steering_angle = 0
            else:
                # 转弯行驶
                radius = linear_x / angular_z
                steering_angle = math.atan(wheelbase / radius)
            
            # 限制转向角度
            max_steering_angle = 0.48869  # 约28度
            steering_angle = max(min(steering_angle, max_steering_angle), -max_steering_angle)
            self.limo.EnableCommand()
            rospy.loginfo("已启用命令控制模式")
            self.limo.SetMotionCommand(linear_x, 0, 0, steering_angle)
            rospy.logdebug(f"阿克曼模式控制: 线速度={linear_x:.2f}, 转向角度={steering_angle:.2f}")
            
        elif motion_mode == self.MECANUM_MODE:  # 麦轮模式
            self.limo.EnableMec()
            rospy.loginfo("已启用命令控制模式")
            self.limo.SetMotionCommand(linear_x, angular_z, linear_y, 0)
            rospy.logdebug(f"麦轮模式控制: 线速度x={linear_x:.2f}, 线速度y={linear_y:.2f}, 角速度={angular_z:.2f}")
    
    # ===== IMU功能 =====
    
    def apply_filter(self, new_data, buffer):
        """
        应用滑动平均滤波器
        
        参数:
            new_data: 新数据
            buffer: 缓冲区
            
        返回:
            滤波后的数据
        """
        buffer.pop(0)
        buffer.append(new_data)
        
        # 计算平均值
        avg_x = sum(data[0] for data in buffer) / len(buffer)
        avg_y = sum(data[1] for data in buffer) / len(buffer)
        avg_z = sum(data[2] for data in buffer) / len(buffer)
        
        return (avg_x, avg_y, avg_z)
    
    def publish_imu(self, event):
        """
        发布IMU数据
        
        参数:
            event: 定时器事件
            
        返回:
            无
        """
        if not self.enable_imu:
            return
        
        # 获取IMU数据
        accel_data = self.limo.GetIMUAccelData()
        gyro_data = self.limo.GetIMUGyroData()
        
        # 获取欧拉角数据
        yaw = self.limo.GetIMUYawData()
        pitch = self.limo.GetIMUPichData()
        roll = self.limo.GetIMURollData()
        euler_data = (yaw, pitch, roll)
        
        # 应用滤波器（如果启用）
        if self.use_filter:
            accel_data = self.apply_filter(accel_data, self.accel_buffer)
            gyro_data = self.apply_filter(gyro_data, self.gyro_buffer)
            euler_data = self.apply_filter(euler_data, self.euler_buffer)
        
        # 创建IMU消息
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = self.imu_frame_id
        
        # 设置线性加速度（单位：m/s^2）
        imu_msg.linear_acceleration = Vector3(
            accel_data[0],  # x轴加速度
            accel_data[1],  # y轴加速度
            accel_data[2]   # z轴加速度
        )
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance
        
        # 设置角速度（单位：rad/s）
        imu_msg.angular_velocity = Vector3(
            gyro_data[0],  # x轴角速度
            gyro_data[1],  # y轴角速度
            gyro_data[2]   # z轴角速度
        )
        imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
        
        # 将欧拉角转换为四元数
        yaw, pitch, roll = euler_data
        quaternion = tf.transformations.quaternion_from_euler(
            roll * math.pi / 180.0,  # 转换为弧度
            pitch * math.pi / 180.0,
            yaw * math.pi / 180.0
        )
        
        # 设置方向四元数
        imu_msg.orientation = Quaternion(
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3]
        )
        imu_msg.orientation_covariance = self.orientation_covariance
        
        # 发布IMU消息
        self.imu_pub.publish(imu_msg)
        
        # 发布TF变换
        self.tf_broadcaster.sendTransform(
            (0, 0, 0),  # 位置
            quaternion,  # 方向
            rospy.Time.now(),
            self.imu_frame_id,
            "base_link"
        )
        
        # 记录调试信息
        rospy.logdebug(f"IMU数据: 加速度=({accel_data[0]:.2f}, {accel_data[1]:.2f}, {accel_data[2]:.2f}), " +
                      f"角速度=({gyro_data[0]:.2f}, {gyro_data[1]:.2f}, {gyro_data[2]:.2f}), " +
                      f"欧拉角=({roll:.2f}, {pitch:.2f}, {yaw:.2f})")
    
    # ===== 错误检测功能 =====
    
    def check_errors(self, event):
        """
        检查错误状态
        
        参数:
            event: 定时器事件
            
        返回:
            无
        """
        if not self.enable_error_detection:
            return
        
        with self.lock:
            # 获取错误代码
            error_code = self.limo.GetErrorCode()
            
            # 获取电池电压
            battery_voltage = self.limo.GetBatteryVoltage()
            
            # 确定错误状态
            if error_code & 0x01F8:  # 电机或驱动错误
                self.error_state = 3  # 错误
                if not self.emergency_stop:
                    rospy.logwarn("检测到严重错误，触发紧急停止")
                    self.set_emergency_stop(True)
            elif error_code & 0x0002 or battery_voltage < self.battery_critical_threshold:  # 电池严重不足
                self.error_state = 2  # 严重警告
            elif error_code & 0x0001 or battery_voltage < self.battery_warning_threshold:  # 电池电量低
                self.error_state = 1  # 警告
            else:
                self.error_state = 0  # 正常
            
            # 自动恢复（如果启用）
            if self.auto_recovery and self.emergency_stop and self.error_state < 3:
                rospy.loginfo("错误已恢复，自动解除紧急停止")
                self.set_emergency_stop(False)
    
    def publish_error_status(self, event):
        """
        发布错误状态
        
        参数:
            event: 定时器事件
            
        返回:
            无
        """
        if not self.enable_error_detection:
            return
        
        with self.lock:
            # 发布错误状态
            self.error_state_pub.publish(Float32(self.error_state))
            
            # 发布紧急停止状态
            self.emergency_stop_pub.publish(Bool(self.emergency_stop))
    
    def set_emergency_stop(self, enable):
        """
        设置紧急停止状态
        
        参数:
            enable: 是否启用紧急停止
            
        返回:
            无
        """
        if not self.enable_error_detection:
            return
        
        with self.lock:
            self.emergency_stop = enable
            
            if enable:
                # 紧急停止：发送零速度命令
                self.limo.SetMotionCommand(0, 0, 0, 0)
                rospy.logwarn("紧急停止已激活")
            else:
                rospy.loginfo("紧急停止已解除")
    
    def emergency_stop_callback(self, req):
        """
        紧急停止服务回调
        
        参数:
            req: 服务请求
            
        返回:
            服务响应
        """
        self.set_emergency_stop(True)
        return TriggerResponse(True, "紧急停止已激活")
    
    def emergency_recovery_callback(self, req):
        """
        紧急恢复服务回调
        
        参数:
            req: 服务请求
            
        返回:
            服务响应
        """
        if self.error_state < 3:  # 只有在非错误状态下才能恢复
            self.set_emergency_stop(False)
            return TriggerResponse(True, "紧急停止已解除")
        else:
            return TriggerResponse(False, "存在严重错误，无法解除紧急停止")
    
    # ===== 键盘控制功能 =====
    
    def print_keyboard_instructions(self):
        """
        打印键盘控制说明
        
        参数:
            无
            
        返回:
            无
        """
        instructions = """
键盘控制说明:
---------------------------
   q    w    e
   a    s    d
   z    x    c

w/x: 增加/减少前进速度
a/d: 增加/减少旋转速度
q/e: 增加/减少左/右横向速度 (仅麦轮模式)
s: 停止
z/c: 左/右横向移动 (仅麦轮模式)
1/2/3: 切换到差速/阿克曼/麦轮模式
空格: 紧急停止
---------------------------
CTRL+C 退出
"""
        print(instructions)
    
    def get_key(self):
        """
        获取键盘按键
        
        参数:
            无
            
        返回:
            按键字符
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def keyboard_listener(self):
        """
        键盘监听线程
        
        参数:
            无
            
        返回:
            无
        """
        while not rospy.is_shutdown():
            key = self.get_key()
            
            # 根据按键更新速度值
            if key == 'w':
                self.linear_x += self.linear_speed_step
                print(f"前进速度增加: {self.linear_x:.2f} m/s")
            elif key == 'x':
                self.linear_x -= self.linear_speed_step
                print(f"前进速度减少: {self.linear_x:.2f} m/s")
            elif key == 'a':
                self.angular_z += self.angular_speed_step
                print(f"旋转速度增加: {self.angular_z:.2f} rad/s")
            elif key == 'd':
                self.angular_z -= self.angular_speed_step
                print(f"旋转速度减少: {self.angular_z:.2f} rad/s")
            elif key == 'q':
                self.linear_y += self.lateral_speed_step
                print(f"左横向速度增加: {self.linear_y:.2f} m/s")
            elif key == 'e':
                self.linear_y -= self.lateral_speed_step
                print(f"右横向速度增加: {self.linear_y:.2f} m/s")
            elif key == 'z':
                self.linear_y = self.lateral_speed_step
                self.linear_x = 0.0
                self.angular_z = 0.0
                print(f"左横向移动: {self.linear_y:.2f} m/s")
            elif key == 'c':
                self.linear_y = -self.lateral_speed_step
                self.linear_x = 0.0
                self.angular_z = 0.0
                print(f"右横向移动: {self.linear_y:.2f} m/s")
            elif key == 's':
                self.linear_x = 0.0
                self.linear_y = 0.0
                self.angular_z = 0.0
                print("停止")
            elif key == '1':
                self.set_motion_mode(self.DIFF_MODE)
                print(f"切换到{self.get_mode_name(self.DIFF_MODE)}")
            elif key == '2':
                self.set_motion_mode(self.ACKERMANN_MODE)
                print(f"切换到{self.get_mode_name(self.ACKERMANN_MODE)}")
            elif key == '3':
                self.set_motion_mode(self.MECANUM_MODE)
                print(f"切换到{self.get_mode_name(self.MECANUM_MODE)}")
            elif key == ' ':
                if self.emergency_stop:
                    self.set_emergency_stop(False)
                    print("解除紧急停止")
                else:
                    self.set_emergency_stop(True)
                    print("激活紧急停止")
            elif key == '\x03':  # CTRL+C
                rospy.signal_shutdown("用户退出")
                break
    
    # ===== 鼠标控制功能 =====
    
    def create_gui(self):
        """
        创建图形界面
        
        参数:
            无
            
        返回:
            无
        """
        self.root = tk.Tk()
        self.root.title("LIMO小车鼠标控制")
        self.root.geometry("600x400")
        
        # 创建控制区域
        self.control_frame = tk.Frame(self.root, width=600, height=300)
        self.control_frame.pack(fill=tk.BOTH, expand=True)
        
        # 创建控制画布
        self.canvas = tk.Canvas(self.control_frame, bg="white", width=600, height=300)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # 绘制控制区域
        self.canvas.create_oval(150, 50, 450, 250, outline="gray", width=2, tags="joystick_area")
        self.canvas.create_line(300, 50, 300, 250, fill="gray", dash=(4, 4))
        self.canvas.create_line(150, 150, 450, 150, fill="gray", dash=(4, 4))
        
        # 创建操纵杆
        self.joystick = self.canvas.create_oval(290, 140, 310, 160, fill="red", tags="joystick")
        
        # 绑定鼠标事件
        self.canvas.tag_bind("joystick", "<ButtonPress-1>", self.on_joystick_press)
        self.canvas.tag_bind("joystick", "<B1-Motion>", self.on_joystick_drag)
        self.canvas.tag_bind("joystick", "<ButtonRelease-1>", self.on_joystick_release)
        
        # 创建状态显示区域
        self.status_frame = tk.Frame(self.root)
        self.status_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 创建速度显示标签
        tk.Label(self.status_frame, text="前进速度:").grid(row=0, column=0, sticky=tk.W)
        self.linear_x_label = tk.Label(self.status_frame, text="0.00 m/s")
        self.linear_x_label.grid(row=0, column=1, sticky=tk.W)
        
        tk.Label(self.status_frame, text="横向速度:").grid(row=0, column=2, sticky=tk.W)
        self.linear_y_label = tk.Label(self.status_frame, text="0.00 m/s")
        self.linear_y_label.grid(row=0, column=3, sticky=tk.W)
        
        tk.Label(self.status_frame, text="旋转速度:").grid(row=0, column=4, sticky=tk.W)
        self.angular_z_label = tk.Label(self.status_frame, text="0.00 rad/s")
        self.angular_z_label.grid(row=0, column=5, sticky=tk.W)
        
        # 创建模式选择区域
        self.mode_frame = tk.Frame(self.root)
        self.mode_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 创建模式选择按钮
        self.mode_var = tk.IntVar(value=self.current_mode)
        tk.Label(self.mode_frame, text="控制模式:").pack(side=tk.LEFT)
        tk.Radiobutton(self.mode_frame, text="差速模式", variable=self.mode_var, value=0, command=self.on_mode_change).pack(side=tk.LEFT)
        tk.Radiobutton(self.mode_frame, text="阿克曼模式", variable=self.mode_var, value=1, command=self.on_mode_change).pack(side=tk.LEFT)
        tk.Radiobutton(self.mode_frame, text="麦轮模式", variable=self.mode_var, value=2, command=self.on_mode_change).pack(side=tk.LEFT)
        
        # 创建停止按钮
        self.stop_button = tk.Button(self.mode_frame, text="紧急停止", bg="red", fg="white", command=self.emergency_stop_toggle)
        self.stop_button.pack(side=tk.RIGHT, padx=10)
        
        # 启动GUI主循环
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()
    
    def on_joystick_press(self, event):
        """
        操纵杆按下事件
        
        参数:
            event: 事件对象
            
        返回:
            无
        """
        self.canvas.tag_raise("joystick")
    
    def on_joystick_drag(self, event):
        """
        操纵杆拖动事件
        
        参数:
            event: 事件对象
            
        返回:
            无
        """
        # 获取鼠标位置
        x = event.x
        y = event.y
        
        # 限制在操纵杆区域内
        x = max(150, min(x, 450))
        y = max(50, min(y, 250))
        
        # 移动操纵杆
        self.canvas.coords("joystick", x-10, y-10, x+10, y+10)
        
        # 计算速度值
        self.linear_x = -((y - 150) / 100.0) * self.max_linear_speed
        
        # 根据不同模式计算其他速度值
        mode = self.mode_var.get()
        if mode == 0:  # 差速模式
            self.angular_z = -((x - 300) / 150.0) * self.max_angular_speed
            self.linear_y = 0.0
        elif mode == 1:  # 阿克曼模式
            # 在阿克曼模式下，横向移动转换为转向角
            self.angular_z = -((x - 300) / 150.0) * self.max_angular_speed
            self.linear_y = 0.0
        elif mode == 2:  # 麦轮模式
            self.angular_z = 0.0
            self.linear_y = ((x - 300) / 150.0) * self.max_lateral_speed
        
        # 更新速度显示
        self.update_speed_display()
    
    def on_joystick_release(self, event):
        """
        操纵杆释放事件
        
        参数:
            event: 事件对象
            
        返回:
            无
        """
        # 重置操纵杆位置
        self.canvas.coords("joystick", 290, 140, 310, 160)
        
        # 重置速度值
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # 更新速度显示
        self.update_speed_display()
    
    def on_mode_change(self):
        """
        模式切换事件
        
        参数:
            无
            
        返回:
            无
        """
        mode = self.mode_var.get()
        self.set_motion_mode(mode)
    
    def emergency_stop_toggle(self):
        """
        紧急停止切换
        
        参数:
            无
            
        返回:
            无
        """
        if self.emergency_stop:
            self.set_emergency_stop(False)
            self.stop_button.config(text="紧急停止")
        else:
            self.set_emergency_stop(True)
            self.stop_button.config(text="解除紧急停止")
    
    def update_speed_display(self):
        """
        更新速度显示
        
        参数:
            无
            
        返回:
            无
        """
        self.linear_x_label.config(text=f"{self.linear_x:.2f} m/s")
        self.linear_y_label.config(text=f"{self.linear_y:.2f} m/s")
        self.angular_z_label.config(text=f"{self.angular_z:.2f} rad/s")
    
    def on_closing(self):
        """
        窗口关闭事件
        
        参数:
            无
            
        返回:
            无
        """
        rospy.signal_shutdown("用户关闭窗口")
        self.root.destroy()
    
    # ===== 通用控制功能 =====
    
    def publish_cmd_vel(self, event):
        """
        发布速度命令
        
        参数:
            event: 定时器事件
            
        返回:
            无
        """
        if self.control_type not in ['keyboard', 'mouse']:
            return
        
        # 如果错误检测功能已启用且紧急停止已激活，则忽略速度命令
        if self.enable_error_detection and self.emergency_stop:
            return
        
        # 如果错误检测功能已启用，则应用速度限制
        linear_x = self.linear_x
        linear_y = self.linear_y
        angular_z = self.angular_z
        
        if self.enable_error_detection:
            # 根据错误状态调整速度
            if self.error_state == 2:  # 严重警告
                max_speed = self.critical_max_speed
            elif self.error_state == 1:  # 警告
                max_speed = self.warning_max_speed
            else:  # 正常
                max_speed = self.normal_max_speed
            
            # 限制线速度
            linear_x_scale = abs(linear_x) / max(abs(linear_x), 0.001)
            linear_y_scale = abs(linear_y) / max(abs(linear_y), 0.001)
            
            if linear_x_scale > 1.0 or linear_y_scale > 1.0:
                scale = min(1.0 / linear_x_scale, 1.0 / linear_y_scale)
                linear_x *= scale * max_speed / self.normal_max_speed
                linear_y *= scale * max_speed / self.normal_max_speed
                angular_z *= scale  # 同比例缩放角速度
        
        # 发送速度命令
        self.send_motion_command(linear_x, linear_y, angular_z)

def main():
    """
    主函数
    
    参数:
        无
        
    返回:
        无
    """
    try:
        controller = LimoIntegratedController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")

if __name__ == '__main__':
    main()
