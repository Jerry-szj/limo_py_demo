#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
LIMO小车IMU数据发布器

该节点读取LIMO小车的IMU数据（加速度、陀螺仪、欧拉角），
并将其转换为标准ROS IMU消息进行发布。同时提供数据滤波和可视化功能。

作者: Manus
日期: 2025-04-01
"""

import rospy
import math
import numpy as np
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from pylimo.limo import LIMO

class LimoIMUPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('limo_imu_publisher', anonymous=True)
        
        # 获取ROS参数
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.rate = rospy.get_param('~rate', 50.0)  # 发布频率，默认50Hz
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyTHS1')
        self.baud_rate = rospy.get_param('~baud_rate', '460800')
        self.use_filter = rospy.get_param('~use_filter', True)  # 是否使用滤波
        self.filter_size = rospy.get_param('~filter_size', 5)  # 滤波窗口大小
        
        # 初始化LIMO对象
        try:
            self.limo = LIMO(name=self.serial_port, baudrate=self.baud_rate)
            rospy.loginfo("成功连接到LIMO小车")
        except Exception as e:
            rospy.logerr(f"连接LIMO小车失败: {e}")
            return
        
        # 初始化IMU发布器
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=50)
        
        # 初始化TF广播器
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 初始化滤波器缓冲区
        if self.use_filter:
            self.accel_buffer = []
            self.gyro_buffer = []
            self.euler_buffer = []
            for _ in range(self.filter_size):
                self.accel_buffer.append((0.0, 0.0, 0.0))
                self.gyro_buffer.append((0.0, 0.0, 0.0))
                self.euler_buffer.append((0.0, 0.0, 0.0))
        
        # 启动IMU发布循环
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.publish_imu)
        
        # 协方差矩阵（这些值需要根据实际传感器性能调整）
        self.orientation_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        self.angular_velocity_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        self.linear_acceleration_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        
        rospy.loginfo("LIMO IMU发布器已启动")
    
    def apply_filter(self, new_data, buffer):
        """应用滑动平均滤波器"""
        buffer.pop(0)
        buffer.append(new_data)
        
        # 计算平均值
        avg_x = sum(data[0] for data in buffer) / len(buffer)
        avg_y = sum(data[1] for data in buffer) / len(buffer)
        avg_z = sum(data[2] for data in buffer) / len(buffer)
        
        return (avg_x, avg_y, avg_z)
    
    def publish_imu(self, event):
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
        imu_msg.header.frame_id = self.frame_id
        
        # 设置线性加速度（单位：m/s^2）
        # 注意：LIMO的IMU数据单位可能需要转换
        imu_msg.linear_acceleration = Vector3(
            accel_data[0],  # x轴加速度
            accel_data[1],  # y轴加速度
            accel_data[2]   # z轴加速度
        )
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance
        
        # 设置角速度（单位：rad/s）
        # 注意：LIMO的IMU数据单位可能需要转换
        imu_msg.angular_velocity = Vector3(
            gyro_data[0],  # x轴角速度
            gyro_data[1],  # y轴角速度
            gyro_data[2]   # z轴角速度
        )
        imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
        
        # 将欧拉角转换为四元数
        # 注意：欧拉角顺序可能需要调整
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
            self.frame_id,
            "base_link"
        )
        
        # 记录调试信息
        rospy.logdebug(f"IMU数据: 加速度=({accel_data[0]:.2f}, {accel_data[1]:.2f}, {accel_data[2]:.2f}), " +
                      f"角速度=({gyro_data[0]:.2f}, {gyro_data[1]:.2f}, {gyro_data[2]:.2f}), " +
                      f"欧拉角=({roll:.2f}, {pitch:.2f}, {yaw:.2f})")

if __name__ == '__main__':
    try:
        imu_publisher = LimoIMUPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
