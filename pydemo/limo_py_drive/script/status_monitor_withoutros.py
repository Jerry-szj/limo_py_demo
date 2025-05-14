#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Demo 2: LIMO状态信息监控器
功能：获取并显示LIMO小车的状态信息，包括：
1. 车辆状态（VehicleState）
2. 控制模式（ControlMode）
3. 电池电压（BatteryVoltage）
4. 错误代码（ErrorCode）
5. 运动模式（MotionMode）

创新点：
1. 实时GUI界面显示状态信息
2. 电池电量可视化
3. 错误状态自动解析和显示
4. 状态历史记录和趋势图
5. 状态变化通知
"""

import sys
import time
import threading
import csv
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from datetime import datetime
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# 导入pylimo库
sys.path.append('/home/ubuntu/upload')
from pylimo.limo import LIMO
import pylimo.limomsg as limomsg

class StatusMonitor:
    def __init__(self, limo_device=None, log_data=True):
        """初始化状态监控器
        
        Args:
            limo_device: LIMO设备实例，如果为None则创建新实例
            log_data: 是否记录数据到CSV文件
        """
        # 初始化LIMO设备
        self.limo = limo_device if limo_device else LIMO()
        
        # 启用命令模式
        self.limo.EnableCommand()
        print("命令模式已启用")
        
        # 状态数据
        self.vehicle_state = 0
        self.control_mode = 0
        self.battery_voltage = 0.0
        self.error_code = 0
        self.motion_mode = 0
        
        # 运动状态数据
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.lateral_velocity = 0.0
        self.steering_angle = 0.0
        
        # 数据记录
        self.log_data = log_data
        self.csv_file = None
        self.csv_writer = None
        self.data_lock = threading.Lock()
        
        # 历史数据
        self.timestamps = []
        self.battery_history = []
        self.velocity_history = []
        self.error_history = []
        
        # 状态解析字典
        self.vehicle_state_dict = {
            0: "正常",
            1: "紧急停止",
            2: "故障"
        }
        
        self.control_mode_dict = {
            0: "遥控模式",
            1: "命令模式",
            2: "未知模式"
        }
        
        self.motion_mode_dict = {
            0: "差速模式",
            1: "阿克曼模式",
            2: "麦克纳姆轮模式",
            3: "未知模式"
        }
        
        self.error_code_dict = {
            0x0001: "电池电量低",
            0x0002: "电池电量低",
            0x0004: "遥控器连接丢失",
            0x0008: "电机驱动器1错误",
            0x0010: "电机驱动器2错误",
            0x0020: "电机驱动器3错误",
            0x0040: "电机驱动器4错误",
            0x0100: "驱动状态错误"
        }
        
        # 初始化数据记录
        if self.log_data:
            self._init_data_logging()
        
        # 创建GUI界面
        self._create_gui()
        
        # 启动数据更新线程
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop)
        self.update_thread.daemon = True
        self.update_thread.start()
    
    def _init_data_logging(self):
        """初始化数据记录"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = "status_logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        filename = f"{log_dir}/status_{timestamp}.csv"
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'vehicle_state', 'control_mode', 'battery_voltage', 
            'error_code', 'motion_mode', 'linear_velocity', 'angular_velocity',
            'lateral_velocity', 'steering_angle'
        ])
        print(f"数据记录已启动，文件：{filename}")
    
    def _create_gui(self):
        """创建GUI界面"""
        self.root = tk.Tk()
        self.root.title("LIMO状态监控器")
        self.root.geometry("800x600")
        self.root.protocol("WM_DELETE_WINDOW", self.stop)
        
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 创建状态信息框架
        status_frame = ttk.LabelFrame(main_frame, text="状态信息", padding="10")
        status_frame.pack(fill=tk.X, pady=5)
        
        # 创建状态标签
        self.vehicle_state_var = tk.StringVar(value="车辆状态: 未知")
        self.control_mode_var = tk.StringVar(value="控制模式: 未知")
        self.battery_voltage_var = tk.StringVar(value="电池电压: 0.0V")
        self.motion_mode_var = tk.StringVar(value="运动模式: 未知")
        
        ttk.Label(status_frame, textvariable=self.vehicle_state_var, font=("Arial", 12)).grid(row=0, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(status_frame, textvariable=self.control_mode_var, font=("Arial", 12)).grid(row=0, column=1, sticky="w", padx=5, pady=2)
        ttk.Label(status_frame, textvariable=self.battery_voltage_var, font=("Arial", 12)).grid(row=1, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(status_frame, textvariable=self.motion_mode_var, font=("Arial", 12)).grid(row=1, column=1, sticky="w", padx=5, pady=2)
        
        # 创建电池电量进度条
        battery_frame = ttk.LabelFrame(main_frame, text="电池电量", padding="10")
        battery_frame.pack(fill=tk.X, pady=5)
        
        self.battery_progress = ttk.Progressbar(battery_frame, orient="horizontal", length=780, mode="determinate")
        self.battery_progress.pack(fill=tk.X)
        
        # 创建错误信息框架
        error_frame = ttk.LabelFrame(main_frame, text="错误信息", padding="10")
        error_frame.pack(fill=tk.X, pady=5)
        
        self.error_text = tk.Text(error_frame, height=4, width=80, font=("Arial", 10))
        self.error_text.pack(fill=tk.X)
        self.error_text.config(state=tk.DISABLED)
        
        # 创建运动状态框架
        motion_frame = ttk.LabelFrame(main_frame, text="运动状态", padding="10")
        motion_frame.pack(fill=tk.X, pady=5)
        
        self.linear_velocity_var = tk.StringVar(value="线速度: 0.0 m/s")
        self.angular_velocity_var = tk.StringVar(value="角速度: 0.0 rad/s")
        self.lateral_velocity_var = tk.StringVar(value="横向速度: 0.0 m/s")
        self.steering_angle_var = tk.StringVar(value="转向角: 0.0 rad")
        
        ttk.Label(motion_frame, textvariable=self.linear_velocity_var, font=("Arial", 12)).grid(row=0, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(motion_frame, textvariable=self.angular_velocity_var, font=("Arial", 12)).grid(row=0, column=1, sticky="w", padx=5, pady=2)
        ttk.Label(motion_frame, textvariable=self.lateral_velocity_var, font=("Arial", 12)).grid(row=1, column=0, sticky="w", padx=5, pady=2)
        ttk.Label(motion_frame, textvariable=self.steering_angle_var, font=("Arial", 12)).grid(row=1, column=1, sticky="w", padx=5, pady=2)
        
        # 创建图表框架
        chart_frame = ttk.LabelFrame(main_frame, text="状态历史", padding="10")
        chart_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # 创建图表
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 4))
        
        # 电池电压图表
        self.ax1.set_title('电池电压历史')
        self.ax1.set_ylabel('电压 (V)')
        self.ax1.grid(True)
        self.battery_line, = self.ax1.plot([], [], 'b-')
        
        # 速度图表
        self.ax2.set_title('速度历史')
        self.ax2.set_xlabel('时间 (s)')
        self.ax2.set_ylabel('速度 (m/s)')
        self.ax2.grid(True)
        self.velocity_line, = self.ax2.plot([], [], 'g-')
        
        self.fig.tight_layout()
        
        # 将图表嵌入到Tkinter界面
        canvas = FigureCanvasTkAgg(self.fig, master=chart_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 启动GUI更新
        self.root.after(100, self._update_gui)
    
    def _update_status(self):
        """更新状态信息"""
        try:
            # 获取车辆状态
            self.vehicle_state = limomsg.GetVehicleState()
            
            # 获取控制模式
            self.control_mode = limomsg.GetControlMode()
            
            # 获取电池电压
            self.battery_voltage = limomsg.GetBatteryVoltage()
            
            # 获取错误代码
            self.error_code = limomsg.GetErrorCode()
            
            # 获取运动模式
            self.motion_mode = limomsg.GetMotionMode()
            
            # 获取运动状态
            self.linear_velocity = limomsg.GetLinearVelocity()
            self.angular_velocity = limomsg.GetAngularVelocity()
            self.lateral_velocity = limomsg.GetLateralVelocity()
            self.steering_angle = limomsg.GetSteeringAngle()
            
            # 记录历史数据
            with self.data_lock:
                current_time = time.time()
                self.timestamps.append(current_time)
                self.battery_history.append(self.battery_voltage)
                self.velocity_history.append(self.linear_velocity)
                self.error_history.append(self.error_code)
                
                # 限制历史数据长度
                max_history = 100
                if len(self.timestamps) > max_history:
                    self.timestamps = self.timestamps[-max_history:]
                    self.battery_history = self.battery_history[-max_history:]
                    self.velocity_history = self.velocity_history[-max_history:]
                    self.error_history = self.error_history[-max_history:]
            
            # 记录数据到CSV文件
            if self.log_data and self.csv_writer:
                self.csv_writer.writerow([
                    time.time(), self.vehicle_state, self.control_mode, 
                    self.battery_voltage, self.error_code, self.motion_mode,
                    self.linear_velocity, self.angular_velocity,
                    self.lateral_velocity, self.steering_angle
                ])
            
            return True
        except Exception as e:
            print(f"更新状态信息时出错: {e}")
            return False
    
    def _update_gui(self):
        """更新GUI界面"""
        # 更新状态标签
        vehicle_state_str = self.vehicle_state_dict.get(self.vehicle_state, "未知")
        self.vehicle_state_var.set(f"车辆状态: {vehicle_state_str}")
        
        control_mode_str = self.control_mode_dict.get(self.control_mode, "未知")
        self.control_mode_var.set(f"控制模式: {control_mode_str}")
        
        self.battery_voltage_var.set(f"电池电压: {self.battery_voltage:.1f}V")
        
        motion_mode_str = self.motion_mode_dict.get(self.motion_mode, "未知")
        self.motion_mode_var.set(f"运动模式: {motion_mode_str}")
        
        # 更新电池电量进度条
        # 假设电池电压范围为10V-16.8V
        battery_min = 10.0
        battery_max = 16.8
        battery_percentage = max(0, min(100, (self.battery_voltage - battery_min) / (battery_max - battery_min) * 100))
        self.battery_progress["value"] = battery_percentage
        
        # 更新错误信息
        self.error_text.config(state=tk.NORMAL)
        self.error_text.delete(1.0, tk.END)
        
        if self.error_code == 0:
            self.error_text.insert(tk.END, "无错误\n")
        else:
            for code, description in self.error_code_dict.items():
                if self.error_code & code:
                    self.error_text.insert(tk.END, f"- {description}\n")
        
        self.error_text.config(state=tk.DISABLED)
        
        # 更新运动状态
        self.linear_velocity_var.set(f"线速度: {self.linear_velocity:.2f} m/s")
        self.angular_velocity_var.set(f"角速度: {self.angular_velocity:.2f} rad/s")
        self.lateral_velocity_var.set(f"横向速度: {self.lateral_velocity:.2f} m/s")
        self.steering_angle_var.set(f"转向角: {self.steering_angle:.2f} rad")
        
        # 更新图表
        with self.data_lock:
            if len(self.timestamps) > 1:
                rel_timestamps = [t - self.timestamps[0] for t in self.timestamps]
                
                # 更新电池电压图表
                self.battery_line.set_data(rel_timestamps, self.battery_history)
                self.ax1.set_xlim(0, max(rel_timestamps))
                self.ax1.set_ylim(min(self.battery_history) - 0.5, max(self.battery_history) + 0.5)
                
                # 更新速度图表
                self.velocity_line.set_data(rel_timestamps, self.velocity_history)
                self.ax2.set_xlim(0, max(rel_timestamps))
                self.ax2.set_ylim(min(self.velocity_history) - 0.5, max(self.velocity_history) + 0.5)
                
                self.fig.canvas.draw_idle()
        
        # 安排下一次更新
        if self.running:
            self.root.after(100, self._update_gui)
    
    def _update_loop(self):
        """数据更新循环"""
        while self.running:
            self._update_status()
            time.sleep(0.1)  # 更新频率10Hz
    
    def print_current_state(self):
        """打印当前状态"""
        vehicle_state_str = self.vehicle_state_dict.get(self.vehicle_state, "未知")
        control_mode_str = self.control_mode_dict.get(self.control_mode, "未知")
        motion_mode_str = self.motion_mode_dict.get(self.motion_mode, "未知")
        
        print("\n当前LIMO状态:")
        print(f"车辆状态: {vehicle_state_str} ({self.vehicle_state})")
        print(f"控制模式: {control_mode_str} ({self.control_mode})")
        print(f"电池电压: {self.battery_voltage:.1f}V")
        print(f"运动模式: {motion_mode_str} ({self.motion_mode})")
        
        print("\n错误信息:")
        if self.error_code == 0:
            print("无错误")
        else:
            for code, description in self.error_code_dict.items():
                if self.error_code & code:
                    print(f"- {description}")
        
        print("\n运动状态:")
        print(f"线速度: {self.linear_velocity:.2f} m/s")
        print(f"角速度: {self.angular_velocity:.2f} rad/s")
        print(f"横向速度: {self.lateral_velocity:.2f} m/s")
        print(f"转向角: {self.steering_angle:.2f} rad")
    
    def run(self):
        """运行GUI界面"""
        self.root.mainloop()
    
    def stop(self):
        """停止状态监控器"""
        self.running = False
        
        if self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
        
        if self.csv_file:
            self.csv_file.close()
            print("数据记录已停止")
        
        self.root.destroy()
        print("状态监控器已停止")

def main():
    """主函数"""
    print("启动LIMO状态监控器...")
    
    try:
        # 创建状态监控器
        status_monitor = StatusMonitor()
        
        print("\nLIMO状态监控器已启动")
        print("GUI界面已启动，关闭窗口可停止程序")
        
        # 运行GUI界面
        status_monitor.run()
    
    except Exception as e:
        print(f"程序出错: {e}")
    finally:
        if 'status_monitor' in locals():
            status_monitor.stop()
        print("程序已退出")

if __name__ == "__main__":
    main()
