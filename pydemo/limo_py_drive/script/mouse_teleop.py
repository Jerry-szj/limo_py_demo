#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LIMO小车鼠标控制节点

该节点提供鼠标控制LIMO小车的功能，通过图形界面发布速度命令。

作者: Manus
日期: 2025-04-02
"""

import rospy
import tkinter as tk
from geometry_msgs.msg import Twist
import threading

class MouseTeleop:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('mouse_teleop', anonymous=True)
        
        # 获取ROS参数
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)  # 最大线速度
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.0)  # 最大角速度
        self.max_lateral_speed = rospy.get_param('~max_lateral_speed', 0.8)  # 最大横向速度
        
        # 初始化速度值
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # 创建速度命令发布器
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # 创建定时发布线程
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_cmd_vel)
        
        # 创建GUI线程
        self.gui_thread = threading.Thread(target=self.create_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()
        
        rospy.loginfo("鼠标控制节点已启动")
    
    def create_gui(self):
        """创建图形界面"""
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
        self.mode_var = tk.IntVar(value=0)
        tk.Label(self.mode_frame, text="控制模式:").pack(side=tk.LEFT)
        tk.Radiobutton(self.mode_frame, text="差速模式", variable=self.mode_var, value=0, command=self.on_mode_change).pack(side=tk.LEFT)
        tk.Radiobutton(self.mode_frame, text="阿克曼模式", variable=self.mode_var, value=1, command=self.on_mode_change).pack(side=tk.LEFT)
        tk.Radiobutton(self.mode_frame, text="麦轮模式", variable=self.mode_var, value=2, command=self.on_mode_change).pack(side=tk.LEFT)
        
        # 创建停止按钮
        self.stop_button = tk.Button(self.mode_frame, text="紧急停止", bg="red", fg="white", command=self.emergency_stop)
        self.stop_button.pack(side=tk.RIGHT, padx=10)
        
        # 启动GUI主循环
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()
    
    def on_joystick_press(self, event):
        """操纵杆按下事件"""
        self.canvas.tag_raise("joystick")
    
    def on_joystick_drag(self, event):
        """操纵杆拖动事件"""
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
        """操纵杆释放事件"""
        # 重置操纵杆位置
        self.canvas.coords("joystick", 290, 140, 310, 160)
        
        # 重置速度值
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # 更新速度显示
        self.update_speed_display()
    
    def on_mode_change(self):
        """模式切换事件"""
        mode = self.mode_var.get()
        mode_names = ["差速模式", "阿克曼模式", "麦轮模式"]
        rospy.loginfo(f"切换到{mode_names[mode]}")
        
        # 调用ROS服务切换模式
        try:
            from std_srvs.srv import SetBool
            if mode == 0:
                rospy.ServiceProxy('set_diff_mode', SetBool)(True)
            elif mode == 1:
                rospy.ServiceProxy('set_ackermann_mode', SetBool)(True)
            elif mode == 2:
                rospy.ServiceProxy('set_mecanum_mode', SetBool)(True)
        except rospy.ServiceException as e:
            rospy.logwarn(f"模式切换服务调用失败: {e}")
        except Exception as e:
            rospy.logerr(f"模式切换错误: {e}")
    
    def emergency_stop(self):
        """紧急停止"""
        # 重置速度值
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # 重置操纵杆位置
        self.canvas.coords("joystick", 290, 140, 310, 160)
        
        # 更新速度显示
        self.update_speed_display()
        
        rospy.loginfo("紧急停止")
    
    def update_speed_display(self):
        """更新速度显示"""
        self.linear_x_label.config(text=f"{self.linear_x:.2f} m/s")
        self.linear_y_label.config(text=f"{self.linear_y:.2f} m/s")
        self.angular_z_label.config(text=f"{self.angular_z:.2f} rad/s")
    
    def publish_cmd_vel(self, event):
        """发布速度命令"""
        twist = Twist()
        twist.linear.x = self.linear_x
        twist.linear.y = self.linear_y
        twist.angular.z = self.angular_z
        
        self.cmd_vel_pub.publish(twist)
    
    def on_closing(self):
        """窗口关闭事件"""
        rospy.signal_shutdown("用户关闭窗口")
        self.root.destroy()

if __name__ == '__main__':
    try:
        teleop = MouseTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
