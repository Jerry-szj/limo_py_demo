#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LIMO小车麦轮控制模式切换器

该节点实现LIMO小车不同运动模式之间的切换（差速、阿克曼、麦轮），
并提供基于麦轮模式的全向运动控制功能。同时创建模式切换服务和
安全检查机制。

作者: Manus
日期: 2025-04-01
"""

import rospy
import math
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from pylimo.limo import LIMO
from pylimo import limomsg

class LimoModeController:
    def __init__(self):
       
        rospy.init_node('limo_mode_controller', anonymous=True)
        
       
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyTHS0')
        self.baud_rate = rospy.get_param('~baud_rate', '460800')
        self.default_mode = rospy.get_param('~default_mode', 0)  # 默认差
        
     
        self.DIFF_MODE = 0    # 差
        self.ACKERMANN_MODE = 1  # 阿
        self.MECANUM_MODE = 2  # 麦
        
    
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0) 
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.0)  
        self.max_lateral_speed = rospy.get_param('~max_lateral_speed', 0.8)     
        try:
            self.limo = LIMO(name=self.serial_port, baudrate=self.baud_rate)
            rospy.loginfo("成功连接到LIMO小车")
            
            self.limo.EnableCommand()
            rospy.loginfo("已启用命令控制模式")
            
        except Exception as e:
            rospy.logerr(f"连接LIMO失败: {e}")
            return
        
        
        self.diff_mode_service = rospy.Service('set_diff_mode', SetBool, self.set_diff_mode_callback)
        self.ackermann_mode_service = rospy.Service('set_ackermann_mode', SetBool, self.set_ackermann_mode_callback)
        self.mecanum_mode_service = rospy.Service('set_mecanum_mode', SetBool, self.set_mecanum_mode_callback)
        
       
        self.mode_pub = rospy.Publisher('limo_mode', Int8, queue_size=10)
        
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        self.current_mode = self.default_mode
        self.set_motion_mode(self.current_mode)
        
        
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_mode)
        
        rospy.loginfo(f"LIMO麦轮控制模式切换器已启动，当前模式: {self.get_mode_name(self.current_mode)}")
    
    def get_mode_name(self, mode):
        """获取模式名称"""
        if mode == self.DIFF_MODE:
            return "差速模式"
        elif mode == self.ACKERMANN_MODE:
            return "阿克曼模式"
        elif mode == self.MECANUM_MODE:
            return "麦轮模式"
        else:
            return "未知模式"
    
    def set_motion_mode(self, mode):
        limomsg.SetMotionMode(mode)
        self.current_mode = mode
        
        
        if mode == self.MECANUM_MODE:
            try:
                self.limo.EnableMec()
                rospy.loginfo("已发送麦轮模式切换命令")
            except Exception as e:
                rospy.logerr(f"切换到麦轮模式失败: {e}")
        
        rospy.loginfo(f"已切换到{self.get_mode_name(mode)}")
        
        self.limo.SetMotionCommand(0, 0, 0, 0)
    
    def set_diff_mode_callback(self, req):
        if req.data:
            self.set_motion_mode(self.DIFF_MODE)
            return SetBoolResponse(True, f"已切换到{self.get_mode_name(self.DIFF_MODE)}")
        return SetBoolResponse(False, "操作被取消")
    
    def set_ackermann_mode_callback(self, req):
        if req.data:
            self.set_motion_mode(self.ACKERMANN_MODE)
            return SetBoolResponse(True, f"已切换到{self.get_mode_name(self.ACKERMANN_MODE)}")
        return SetBoolResponse(False, "操作被取消")
    
    def set_mecanum_mode_callback(self, req):
        if req.data:
            self.set_motion_mode(self.MECANUM_MODE)
            return SetBoolResponse(True, f"已切换到{self.get_mode_name(self.MECANUM_MODE)}")
        return SetBoolResponse(False, "操作被取消")
    
    def publish_mode(self, event):
        self.mode_pub.publish(Int8(self.current_mode))
    
    def cmd_vel_callback(self, msg):

        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        
        linear_x = self.clamp(linear_x, -self.max_linear_speed, self.max_linear_speed)
        linear_y = self.clamp(linear_y, -self.max_lateral_speed, self.max_lateral_speed)
        angular_z = self.clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)
        

        if self.current_mode == self.DIFF_MODE:

            self.limo.SetMotionCommand(linear_x, angular_z, 0, 0)
            rospy.logdebug(f"差速模式控制: 线速度={linear_x:.2f}, 角速度={angular_z:.2f}")
            
        elif self.current_mode == self.ACKERMANN_MODE:
        
            if abs(linear_x) < 0.1:
                
                self.limo.SetMotionCommand(0, 0, 0, 0)
                rospy.logdebug("阿克曼模式控制: 速度太小，停止")
            else:
 
                wheelbase = 0.2  
                if abs(angular_z) < 0.001:
            
                    steering_angle = 0
                else:
                
                    radius = linear_x / angular_z
                    steering_angle = math.atan(wheelbase / radius)
        
                max_steering_angle = 0.48869  # 约28度
                steering_angle = self.clamp(steering_angle, -max_steering_angle, max_steering_angle)
                
                self.limo.SetMotionCommand(linear_x, 0, 0, steering_angle)
                rospy.logdebug(f"阿克曼模式控制: 线速度={linear_x:.2f}, 转向角度={steering_angle:.2f}")
                
        elif self.current_mode == self.MECANUM_MODE:
            
            self.limo.SetMotionCommand(linear_x, angular_z, linear_y, 0)
            rospy.logdebug(f"麦轮模式控制: 线速度={linear_x:.2f}, 角速度={angular_z:.2f}, 横向速度={linear_y:.2f}")
    
    def clamp(self, value, min_value, max_value):
      
        return max(min(value, max_value), min_value)

if __name__ == '__main__':
    try:
        controller = LimoModeController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
