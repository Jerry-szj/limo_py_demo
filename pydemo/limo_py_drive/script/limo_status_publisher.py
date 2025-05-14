#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LIMO小车状态信息发布器

该节点读取LIMO小车的状态信息（电池电压、控制模式、车辆状态、错误代码等），
并使用自定义ROS消息类型进行发布。同时提供状态监控和日志记录功能。

作者: Manus
日期: 2025-04-02
"""

import rospy
import std_msgs.msg
from pylimo.limo import LIMO
from pylimo import limomsg

class LimoStatusPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('limo_status_publisher', anonymous=True)
        
        # 获取ROS参数
        self.rate = rospy.get_param('~rate', 1.0)  # 发布频率，默认1Hz
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyTHS1')
        self.baud_rate = rospy.get_param('~baud_rate', '460800')
        
        # 初始化LIMO对象
        try:
            self.limo = LIMO(name=self.serial_port, baudrate=self.baud_rate)
            rospy.loginfo("成功连接到LIMO小车")
        except Exception as e:
            rospy.logerr(f"连接LIMO小车失败: {e}")
            return
        
        # 初始化状态信息发布器 - 使用标准消息类型代替自定义消息类型
        self.battery_pub = rospy.Publisher('limo_status/battery', std_msgs.msg.Float32, queue_size=10)
        self.control_mode_pub = rospy.Publisher('limo_status/control_mode', std_msgs.msg.UInt8, queue_size=10)
        self.vehicle_state_pub = rospy.Publisher('limo_status/vehicle_state', std_msgs.msg.UInt8, queue_size=10)
        self.motion_mode_pub = rospy.Publisher('limo_status/motion_mode', std_msgs.msg.UInt8, queue_size=10)
        self.error_code_pub = rospy.Publisher('limo_status/error_code', std_msgs.msg.UInt16, queue_size=10)
        self.error_desc_pub = rospy.Publisher('limo_status/error_description', std_msgs.msg.String, queue_size=10)
        
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
        
        # 启动状态信息发布循环
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.publish_status)
        
        # 记录上一次的错误代码，用于检测错误状态变化
        self.last_error_code = 0
        
        rospy.loginfo("LIMO状态信息发布器已启动")
    
    def publish_status(self, event):
        # 获取电池电压
        battery_voltage = self.limo.GetBatteryVoltage()
        self.battery_pub.publish(std_msgs.msg.Float32(battery_voltage))
        
        # 获取控制模式
        control_mode = self.limo.GetControlMode()
        self.control_mode_pub.publish(std_msgs.msg.UInt8(control_mode))
        
        # 获取车辆状态
        vehicle_state = limomsg.GetVehicleState()
        self.vehicle_state_pub.publish(std_msgs.msg.UInt8(vehicle_state))
        
        # 获取运动模式
        motion_mode = limomsg.GetMotionMode()
        self.motion_mode_pub.publish(std_msgs.msg.UInt8(motion_mode))
        
        # 获取错误代码
        error_code = self.limo.GetErrorCode()
        self.error_code_pub.publish(std_msgs.msg.UInt16(error_code))
        
        # 解析错误代码并生成错误描述
        error_descriptions = []
        for bit, description in self.error_code_map.items():
            if error_code & bit:
                error_descriptions.append(description)
        
        # 如果没有错误，添加一个"正常"描述
        if not error_descriptions:
            error_descriptions.append("正常")
        
        # 发布错误描述
        self.error_desc_pub.publish(std_msgs.msg.String(", ".join(error_descriptions)))
        
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

if __name__ == '__main__':
    try:
        status_publisher = LimoStatusPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
