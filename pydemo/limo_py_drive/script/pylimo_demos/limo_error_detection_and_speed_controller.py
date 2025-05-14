#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LIMO小车错误检测与速度控制器

该节点实现LIMO小车的错误检测和处理机制，以及基于错误状态的自适应速度控制。
同时提供紧急停止和恢复功能，以及远程控制接口。

作者: Manus
日期: 2025-04-01
"""

import rospy
import math
import threading
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Twist
from pylimo.limo import LIMO
from pylimo import limomsg

class LimoErrorDetectionAndSpeedController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('limo_error_detection_and_speed_controller', anonymous=True)
        
        # 获取ROS参数
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyTHS1')
        self.baud_rate = rospy.get_param('~baud_rate', '460800')
        self.check_rate = rospy.get_param('~check_rate', 10.0)  # 错误检测频率，默认10Hz
        self.auto_recovery = rospy.get_param('~auto_recovery', True)  # 是否自动恢复
        
        # 速度限制参数
        self.normal_max_speed = rospy.get_param('~normal_max_speed', 1.0)  # 正常最大速度 m/s
        self.warning_max_speed = rospy.get_param('~warning_max_speed', 0.5)  # 警告状态最大速度 m/s
        self.critical_max_speed = rospy.get_param('~critical_max_speed', 0.2)  # 严重警告最大速度 m/s
        
        # 电池电压阈值
        self.battery_warning_threshold = rospy.get_param('~battery_warning_threshold', 11.0)  # 电池警告阈值
        self.battery_critical_threshold = rospy.get_param('~battery_critical_threshold', 10.0)  # 电池严重警告阈值
        
        # 初始化LIMO对象
        try:
            self.limo = LIMO(name=self.serial_port, baudrate=self.baud_rate)
            rospy.loginfo("成功连接到LIMO小车")
            
            # 启用命令控制模式
            self.limo.EnableCommand()
            rospy.loginfo("已启用命令控制模式")
            
        except Exception as e:
            rospy.logerr(f"连接LIMO小车失败: {e}")
            return
        
        # 初始化状态变量
        self.error_state = 0  # 0: 正常, 1: 警告, 2: 严重警告, 3: 错误
        self.emergency_stop = False  # 紧急停止标志
        self.last_cmd_vel = Twist()  # 上一次的速度命令
        self.last_error_code = 0  # 上一次的错误代码
        
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
        
        # 创建锁，用于线程同步
        self.lock = threading.Lock()
        
        # 创建发布器
        self.error_state_pub = rospy.Publisher('limo_error_state', Float32, queue_size=10)
        self.emergency_stop_pub = rospy.Publisher('limo_emergency_stop', Bool, queue_size=10)
        
        # 创建服务
        self.emergency_stop_srv = rospy.Service('emergency_stop', Trigger, self.emergency_stop_callback)
        self.emergency_recovery_srv = rospy.Service('emergency_recovery', Trigger, self.emergency_recovery_callback)
        
        # 创建订阅器
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        # 创建错误检测定时器
        self.error_check_timer = rospy.Timer(rospy.Duration(1.0/self.check_rate), self.check_errors)
        
        # 创建状态发布定时器
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        rospy.loginfo("LIMO错误检测与速度控制器已启动")
    
    def check_errors(self, event):
        """检查错误状态"""
        with self.lock:
            # 获取错误代码
            error_code = self.limo.GetErrorCode()
            
            # 获取电池电压
            battery_voltage = self.limo.GetBatteryVoltage()
            
            # 检测错误状态变化
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
    
    def publish_status(self, event):
        """发布状态信息"""
        with self.lock:
            # 发布错误状态
            self.error_state_pub.publish(Float32(self.error_state))
            
            # 发布紧急停止状态
            self.emergency_stop_pub.publish(Bool(self.emergency_stop))
    
    def set_emergency_stop(self, enable):
        """设置紧急停止状态"""
        with self.lock:
            self.emergency_stop = enable
            
            if enable:
                # 紧急停止：发送零速度命令
                self.limo.SetMotionCommand(0, 0, 0, 0)
                rospy.logwarn("紧急停止已激活")
            else:
                rospy.loginfo("紧急停止已解除")
    
    def emergency_stop_callback(self, req):
        """紧急停止服务回调"""
        self.set_emergency_stop(True)
        return TriggerResponse(True, "紧急停止已激活")
    
    def emergency_recovery_callback(self, req):
        """紧急恢复服务回调"""
        if self.error_state < 3:  # 只有在非错误状态下才能恢复
            self.set_emergency_stop(False)
            return TriggerResponse(True, "紧急停止已解除")
        else:
            return TriggerResponse(False, "存在严重错误，无法解除紧急停止")
    
    def cmd_vel_callback(self, msg):
        """速度命令回调"""
        with self.lock:
            # 保存速度命令
            self.last_cmd_vel = msg
            
            # 如果紧急停止已激活，则忽略速度命令
            if self.emergency_stop:
                rospy.logdebug("紧急停止已激活，忽略速度命令")
                return
            
            # 根据错误状态调整速度
            linear_x = msg.linear.x
            linear_y = msg.linear.y
            angular_z = msg.angular.z
            
            # 应用速度限制
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
            
            # 发送调整后的速度命令
            motion_mode = limomsg.GetMotionMode()
            
            if motion_mode == 0:  # 差速模式
                self.limo.SetMotionCommand(linear_x, angular_z, 0, 0)
            elif motion_mode == 1:  # 阿克曼模式
                # 计算转向角度
                wheelbase = 0.2  # 轴距
                if abs(linear_x) < 0.1 or abs(angular_z) < 0.001:
                    steering_angle = 0
                else:
                    radius = linear_x / angular_z
                    steering_angle = math.atan(wheelbase / radius)
                
                # 限制转向角度
                max_steering_angle = 0.48869  # 约28度
                steering_angle = max(min(steering_angle, max_steering_angle), -max_steering_angle)
                
                self.limo.SetMotionCommand(linear_x, 0, 0, steering_angle)
            elif motion_mode == 2:  # 麦轮模式
                self.limo.SetMotionCommand(linear_x, angular_z, linear_y, 0)
            
            rospy.logdebug(f"发送速度命令: 线速度x={linear_x:.2f}, 线速度y={linear_y:.2f}, 角速度={angular_z:.2f}")

if __name__ == '__main__':
    try:
        controller = LimoErrorDetectionAndSpeedController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
