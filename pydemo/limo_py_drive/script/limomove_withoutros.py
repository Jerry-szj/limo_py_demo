#!/usr/bin/env python3
# coding=UTF-8

#使用enableCommandedMode()使能控制模式、setMotionCommand()	设置limo的控制模式
from pylimo import LIMO
import time

def move_robot(direction, duration, linear_vel=0.5, angular_vel=0.5):
    limo = LIMO()
    limo.EnableCommand()  # 使能控制

    try:
        # 定义方向和对应的运动命令
        motion_commands = {
            'forward': (linear_vel, 0.0),
            'backward': (-linear_vel, 0.0),
            'left': (0.0, angular_vel),
            'right': (0.0, -angular_vel)
        }

        # 获取对应的运动命令
        if direction in motion_commands:
            linear, angular = motion_commands[direction]
            print(f'Moving {direction}')
            limo.SetMotionCommand(linear_vel=linear, angular_vel=angular)
            time.sleep(duration)  # 持续时间
        else:
            print('Invalid direction')
    finally:
        limo.SetMotionCommand(linear_vel=0.0, angular_vel=0.0)  # 停止运动
        #limo.close()  # 关闭连接

if __name__ == "__main__":
    # 前后左右各移动1秒
    directions = ['forward', 'backward', 'left', 'right']
    for direction in directions:
        move_robot(direction, duration=1)
