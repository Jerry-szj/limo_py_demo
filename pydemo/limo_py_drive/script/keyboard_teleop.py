#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LIMO小车键盘控制节点

该节点提供键盘控制LIMO小车的功能，通过键盘按键发布速度命令。

作者: Manus
日期: 2025-04-02
"""

import rospy
import sys
import termios
import tty
import threading
from geometry_msgs.msg import Twist

class KeyboardTeleop:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('keyboard_teleop', anonymous=True)
        
        # 获取ROS参数
        self.linear_speed_step = rospy.get_param('~linear_speed_step', 0.1)  # 线速度步进值
        self.angular_speed_step = rospy.get_param('~angular_speed_step', 0.1)  # 角速度步进值
        self.lateral_speed_step = rospy.get_param('~lateral_speed_step', 0.1)  # 横向速度步进值
        
        # 初始化速度值
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # 创建速度命令发布器
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # 显示控制说明
        self.print_instructions()
        
        # 创建键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # 创建定时发布线程
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_cmd_vel)
        
        rospy.loginfo("键盘控制节点已启动")
    
    def print_instructions(self):
        """打印控制说明"""
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
---------------------------
CTRL+C 退出
"""
        print(instructions)
    
    def get_key(self):
        """获取键盘按键"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def keyboard_listener(self):
        """键盘监听线程"""
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
            elif key == '\x03':  # CTRL+C
                rospy.signal_shutdown("用户退出")
                break
    
    def publish_cmd_vel(self, event):
        """发布速度命令"""
        twist = Twist()
        twist.linear.x = self.linear_x
        twist.linear.y = self.linear_y
        twist.angular.z = self.angular_z
        
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        teleop = KeyboardTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
