#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LIMO小车状态信息发布器

该节点读取LIMO小车的状态信息（电池电压、控制模式、车辆状态、错误代码等），
并使用自定义ROS消息类型进行发布。同时提供状态监控和日志记录功能。

作者: Manus
日期: 2025-04-01
"""

import rospy
from pylimo.limo import LIMO
from limo_msgs.msg import LimoStatus

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
        
        # 初始化状态信息发布器
        self.status_pub = rospy.Publisher('limo_status', LimoStatus, queue_size=10)
        
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
        # 创建状态消息
        status_msg = LimoStatus()
        status_msg.header.stamp = rospy.Time.now()
        
        # 获取电池电压
        status_msg.battery_voltage = self.limo.GetBatteryVoltage()
        
        # 获取控制模式
        status_msg.control_mode = self.limo.GetControlMode()
        
        # 获取车辆状态
        status_msg.vehicle_state = limomsg.GetVehicleState()
        
        # 获取运动模式
        status_msg.motion_mode = limomsg.GetMotionMode()
        
        # 获取错误代码
        error_code = self.limo.GetErrorCode()
        status_msg.error_code = error_code
        
        # 解析错误代码并生成错误描述
        error_descriptions = []
        for bit, description in self.error_code_map.items():
            if error_code & bit:
                error_descriptions.append(description)
        
        # 如果没有错误，添加一个"正常"描述
        if not error_descriptions:
            error_descriptions.append("正常")
        
        status_msg.error_descriptions = error_descriptions
        
        # 发布状态消息
        self.status_pub.publish(status_msg)
        
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
        if status_msg.battery_voltage < 10.0:
            rospy.logwarn(f"电池电量严重不足: {status_msg.battery_voltage:.2f}V")
        elif status_msg.battery_voltage < 11.0:
            rospy.logwarn(f"电池电量低: {status_msg.battery_voltage:.2f}V")
        
        # 记录调试信息
        rospy.logdebug(f"电池电压: {status_msg.battery_voltage:.2f}V, 控制模式: {status_msg.control_mode}, " +
                      f"车辆状态: {status_msg.vehicle_state}, 运动模式: {status_msg.motion_mode}, " +
                      f"错误代码: {status_msg.error_code}")

if __name__ == '__main__':
    try:
        # 导入limomsg模块（在类外部导入以避免循环导入问题）
        from pylimo import limomsg
        
        status_publisher = LimoStatusPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
