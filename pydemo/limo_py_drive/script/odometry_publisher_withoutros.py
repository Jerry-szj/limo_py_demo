#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Demo 1: 里程计数据发布器
功能：获取并发布LIMO小车的里程计数据，包括：
1. 左右轮里程计数据获取
2. 计算小车的位置和方向
3. 实时显示里程计数据
4. 数据记录到CSV文件

创新点：
1. 实时可视化里程计轨迹
2. 支持数据记录和回放
3. 计算并显示行驶距离和速度
"""

import sys
import time
import math
import csv
import os
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from datetime import datetime

# 导入pylimo库
sys.path.append('/home/ubuntu/upload')
from pylimo.limo import LIMO
import pylimo.limomsg as limomsg

class OdometryPublisher:
    def __init__(self, limo_device=None, log_data=True):
        """初始化里程计发布器
        
        Args:
            limo_device: LIMO设备实例，如果为None则创建新实例
            log_data: 是否记录数据到CSV文件
        """
        # 初始化LIMO设备
        self.limo = limo_device if limo_device else LIMO()
        
        # 启用命令模式
        self.limo.EnableCommand()
        print("命令模式已启用")
        
        # 初始化里程计数据
        self.left_wheel_odom = 0
        self.right_wheel_odom = 0
        self.prev_left_wheel_odom = 0
        self.prev_right_wheel_odom = 0
        
        # 初始化位置和方向
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 车轮参数
        self.wheel_radius = 0.0325  # 车轮半径，单位：米
        self.wheel_base = 0.2  # 轮距，单位：米
        
        # 数据记录
        self.log_data = log_data
        self.csv_file = None
        self.csv_writer = None
        self.data_lock = threading.Lock()
        
        # 轨迹数据
        self.trajectory_x = []
        self.trajectory_y = []
        self.timestamps = []
        
        # 速度数据
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.distance_traveled = 0.0
        
        # 初始化数据记录
        if self.log_data:
            self._init_data_logging()
        
        # 初始化可视化
        self._init_visualization()
        
        # 启动数据更新线程
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop)
        self.update_thread.daemon = True
        self.update_thread.start()
    
    def _init_data_logging(self):
        """初始化数据记录"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = "odometry_logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        filename = f"{log_dir}/odometry_{timestamp}.csv"
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'left_wheel_odom', 'right_wheel_odom', 
            'x', 'y', 'theta', 'linear_velocity', 'angular_velocity',
            'distance_traveled'
        ])
        print(f"数据记录已启动，文件：{filename}")
    
    def _init_visualization(self):
        """初始化可视化界面"""
        plt.ion()  # 开启交互模式
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # 轨迹图
        self.ax1.set_title('小车轨迹')
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.grid(True)
        self.trajectory_line, = self.ax1.plot([], [], 'b-')
        self.position_point, = self.ax1.plot([], [], 'ro')
        
        # 速度图
        self.ax2.set_title('速度数据')
        self.ax2.set_xlabel('时间 (s)')
        self.ax2.set_ylabel('速度 (m/s)')
        self.ax2.grid(True)
        self.velocity_line, = self.ax2.plot([], [], 'g-')
        
        self.fig.tight_layout()
        plt.show(block=False)
    
    def _update_visualization(self):
        """更新可视化界面"""
        with self.data_lock:
            x_data = self.trajectory_x.copy()
            y_data = self.trajectory_y.copy()
            timestamps = self.timestamps.copy()
            linear_velocity = self.linear_velocity
        
        if not x_data:
            return
        
        # 更新轨迹图
        self.trajectory_line.set_data(x_data, y_data)
        self.position_point.set_data([x_data[-1]], [y_data[-1]])
        
        # 自动调整轨迹图的范围
        if len(x_data) > 1:
            x_min, x_max = min(x_data), max(x_data)
            y_min, y_max = min(y_data), max(y_data)
            
            # 确保x和y轴的比例相同
            x_range = max(x_max - x_min, 0.1)
            y_range = max(y_max - y_min, 0.1)
            max_range = max(x_range, y_range)
            
            x_mid = (x_max + x_min) / 2
            y_mid = (y_max + y_min) / 2
            
            self.ax1.set_xlim(x_mid - max_range/1.8, x_mid + max_range/1.8)
            self.ax1.set_ylim(y_mid - max_range/1.8, y_mid + max_range/1.8)
        
        # 更新速度图
        if len(timestamps) > 1:
            rel_timestamps = [t - timestamps[0] for t in timestamps]
            self.velocity_line.set_data(rel_timestamps, [0] + [self.trajectory_x[i+1] - self.trajectory_x[i] for i in range(len(self.trajectory_x)-1)])
            self.ax2.set_xlim(0, max(rel_timestamps))
            self.ax2.set_ylim(-0.5, 0.5)
        
        # 更新标题显示当前速度和行驶距离
        self.ax1.set_title(f'小车轨迹 (距离: {self.distance_traveled:.2f} m)')
        self.ax2.set_title(f'速度数据 (当前: {linear_velocity:.2f} m/s)')
        
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
    
    def _update_odometry(self):
        """更新里程计数据"""
        try:
            # 获取当前里程计数据
            self.left_wheel_odom = self.limo.GetLeftWheelOdeom()
            self.right_wheel_odom = self.limo.GetRightWheelOdom()
            
            # 计算车轮转动增量（单位：弧度）
            delta_left = (self.left_wheel_odom - self.prev_left_wheel_odom) * 2 * math.pi / 1000.0
            delta_right = (self.right_wheel_odom - self.prev_right_wheel_odom) * 2 * math.pi / 1000.0
            
            # 更新上一次的里程计数据
            self.prev_left_wheel_odom = self.left_wheel_odom
            self.prev_right_wheel_odom = self.right_wheel_odom
            
            # 计算线速度和角速度
            self.linear_velocity = self.wheel_radius * (delta_right + delta_left) / 2.0
            self.angular_velocity = self.wheel_radius * (delta_right - delta_left) / self.wheel_base
            
            # 计算位置和方向变化
            if abs(self.angular_velocity) < 0.0001:
                # 直线运动
                distance = self.linear_velocity
                dx = distance * math.cos(self.theta)
                dy = distance * math.sin(self.theta)
                dtheta = 0.0
            else:
                # 曲线运动
                radius = self.linear_velocity / self.angular_velocity
                dtheta = self.angular_velocity
                dx = radius * (math.sin(self.theta + dtheta) - math.sin(self.theta))
                dy = radius * (math.cos(self.theta) - math.cos(self.theta + dtheta))
            
            # 更新位置和方向
            self.x += dx
            self.y += dy
            self.theta += dtheta
            
            # 归一化角度到[-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            
            # 更新行驶距离
            self.distance_traveled += math.sqrt(dx*dx + dy*dy)
            
            # 记录轨迹数据
            with self.data_lock:
                self.trajectory_x.append(self.x)
                self.trajectory_y.append(self.y)
                self.timestamps.append(time.time())
            
            # 记录数据到CSV文件
            if self.log_data and self.csv_writer:
                self.csv_writer.writerow([
                    time.time(), self.left_wheel_odom, self.right_wheel_odom,
                    self.x, self.y, self.theta, self.linear_velocity, 
                    self.angular_velocity, self.distance_traveled
                ])
            
            return True
        except Exception as e:
            print(f"更新里程计数据时出错: {e}")
            return False
    
    def _update_loop(self):
        """数据更新循环"""
        update_viz_counter = 0
        while self.running:
            if self._update_odometry():
                # 每5次数据更新更新一次可视化（降低CPU负载）
                update_viz_counter += 1
                if update_viz_counter >= 200:
                    self._update_visualization()
                    update_viz_counter = 0
            
            # 控制更新频率
            time.sleep(0.05)
    
    def print_current_state(self):
        """打印当前状态"""
        print("\n当前里程计状态:")
        print(f"左轮里程计: {self.left_wheel_odom}")
        print(f"右轮里程计: {self.right_wheel_odom}")
        print(f"位置: X={self.x:.3f} m, Y={self.y:.3f} m")
        print(f"方向: {math.degrees(self.theta):.1f}°")
        print(f"线速度: {self.linear_velocity:.3f} m/s")
        print(f"角速度: {self.angular_velocity:.3f} rad/s")
        print(f"行驶距离: {self.distance_traveled:.3f} m")
    
    def stop(self):
        """停止里程计发布器"""
        self.running = False
        if self.update_thread.is_alive():
            self.update_thread.join()
        
        if self.csv_file:
            self.csv_file.close()
            print("数据记录已停止")
        
        plt.close(self.fig)
        print("里程计发布器已停止")

def main():
    """主函数"""
    print("启动里程计数据发布器...")
    
    try:
        # 创建里程计发布器
        odometry_publisher = OdometryPublisher()
        
        print("\n里程计数据发布器已启动")
        print("按Ctrl+C停止程序")
        
        # 主循环
        while True:
            # 每秒打印一次当前状态
            odometry_publisher.print_current_state()
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\n检测到Ctrl+C，正在停止程序...")
    except Exception as e:
        print(f"程序出错: {e}")
    finally:
        if 'odometry_publisher' in locals():
            odometry_publisher.stop()
        print("程序已退出")

if __name__ == "__main__":
    main()
