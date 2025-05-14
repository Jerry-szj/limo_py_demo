# LIMO小车Python控制Demo使用说明

## 项目概述

本项目基于pylimo库开发了5个Python控制Agilex LIMO小车的demo，包括里程计数据发布、状态信息发布、麦轮控制模式切换、IMU数据发布和错误检测与速度控制等功能。这些demo展示了如何使用pylimo库与LIMO小车进行通信和控制，并将其集成到ROS环境中。

## 环境要求

- Ubuntu 20.04
- ROS1 (Noetic)
- Python 3.8
- pylimo库（已提供源码）

## 安装步骤

1. 确保已安装ROS1 Noetic，如果没有安装，请参考[ROS安装指南](http://wiki.ros.org/noetic/Installation/Ubuntu)

2. 创建工作空间并克隆代码：

```bash
mkdir -p ~/limo_ws/src
cd ~/limo_ws/src
# 将本项目文件复制到此目录
```

3. 安装依赖：

```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-serial python3-numpy
pip3 install numpy tf transformations
```

4. 安装pylimo库：

```bash
cd ~/limo_ws/src
mkdir -p pylimo/pylimo
cp /path/to/limo.py /path/to/limomsg.py /path/to/__init__.py ~/limo_ws/src/pylimo/pylimo/
touch ~/limo_ws/src/pylimo/__init__.py
cd ~/limo_ws/src/pylimo
pip3 install -e .
```

5. 编译自定义消息：

```bash
cd ~/limo_ws
catkin_make
source devel/setup.bash
```

## Demo说明

### 1. 里程计数据发布器 (limo_odometry_publisher.py)

**功能**：
- 读取LIMO小车的左右轮里程计数据
- 计算并发布标准ROS里程计消息
- 发布TF变换，支持可视化

**使用方法**：
```bash
rosrun pylimo_demos limo_odometry_publisher.py _serial_port:=/dev/ttyTHS1 _baud_rate:=460800 _rate:=10
```

**参数**：
- `_serial_port`：串口设备路径，默认为`/dev/ttyTHS1`
- `_baud_rate`：波特率，默认为`460800`
- `_rate`：发布频率，默认为`10`Hz
- `_frame_id`：里程计坐标系，默认为`odom`
- `_child_frame_id`：小车坐标系，默认为`base_link`

**发布的话题**：
- `/odom`：标准ROS里程计消息

### 2. 状态信息发布器 (limo_status_publisher.py)

**功能**：
- 读取小车状态信息（电池电压、控制模式、车辆状态、错误代码等）
- 使用自定义ROS消息类型发布状态信息
- 提供状态监控和日志记录功能

**使用方法**：
```bash
rosrun pylimo_demos limo_status_publisher.py _serial_port:=/dev/ttyTHS1 _baud_rate:=460800 _rate:=1
```

**参数**：
- `_serial_port`：串口设备路径，默认为`/dev/ttyTHS1`
- `_baud_rate`：波特率，默认为`460800`
- `_rate`：发布频率，默认为`1`Hz

**发布的话题**：
- `/limo_status`：自定义LIMO状态消息

### 3. 麦轮控制模式切换器 (limo_mode_controller.py)

**功能**：
- 实现不同运动模式之间的切换（差速、阿克曼、麦轮）
- 提供模式切换服务
- 实现基于麦轮模式的全向运动控制
- 提供安全检查和限制功能

**使用方法**：
```bash
rosrun pylimo_demos limo_mode_controller.py _serial_port:=/dev/ttyTHS1 _baud_rate:=460800 _default_mode:=0
```

**参数**：
- `_serial_port`：串口设备路径，默认为`/dev/ttyTHS1`
- `_baud_rate`：波特率，默认为`460800`
- `_default_mode`：默认运动模式，0=差速，1=阿克曼，2=麦轮，默认为`0`
- `_max_linear_speed`：最大线速度，默认为`1.0`m/s
- `_max_angular_speed`：最大角速度，默认为`2.0`rad/s
- `_max_lateral_speed`：最大横向速度，默认为`0.8`m/s

**发布的话题**：
- `/limo_mode`：当前运动模式

**订阅的话题**：
- `/cmd_vel`：速度命令

**提供的服务**：
- `/set_diff_mode`：切换到差速模式
- `/set_ackermann_mode`：切换到阿克曼模式
- `/set_mecanum_mode`：切换到麦轮模式

### 4. IMU数据发布器 (limo_imu_publisher.py)

**功能**：
- 读取IMU数据（加速度、陀螺仪、欧拉角）
- 发布标准ROS IMU消息
- 提供数据滤波功能
- 发布TF变换，支持可视化

**使用方法**：
```bash
rosrun pylimo_demos limo_imu_publisher.py _serial_port:=/dev/ttyTHS1 _baud_rate:=460800 _rate:=50 _use_filter:=true
```

**参数**：
- `_serial_port`：串口设备路径，默认为`/dev/ttyTHS1`
- `_baud_rate`：波特率，默认为`460800`
- `_rate`：发布频率，默认为`50`Hz
- `_frame_id`：IMU坐标系，默认为`imu_link`
- `_use_filter`：是否使用滤波，默认为`true`
- `_filter_size`：滤波窗口大小，默认为`5`

**发布的话题**：
- `/imu/data`：标准ROS IMU消息

### 5. 错误检测与速度控制器 (limo_error_detection_and_speed_controller.py)

**功能**：
- 实现错误检测和处理机制
- 提供基于错误状态的自适应速度控制
- 提供紧急停止和恢复功能
- 实现远程控制接口

**使用方法**：
```bash
rosrun pylimo_demos limo_error_detection_and_speed_controller.py _serial_port:=/dev/ttyTHS1 _baud_rate:=460800 _check_rate:=10 _auto_recovery:=true
```

**参数**：
- `_serial_port`：串口设备路径，默认为`/dev/ttyTHS1`
- `_baud_rate`：波特率，默认为`460800`
- `_check_rate`：错误检测频率，默认为`10`Hz
- `_auto_recovery`：是否自动恢复，默认为`true`
- `_normal_max_speed`：正常最大速度，默认为`1.0`m/s
- `_warning_max_speed`：警告状态最大速度，默认为`0.5`m/s
- `_critical_max_speed`：严重警告最大速度，默认为`0.2`m/s
- `_battery_warning_threshold`：电池警告阈值，默认为`11.0`V
- `_battery_critical_threshold`：电池严重警告阈值，默认为`10.0`V

**发布的话题**：
- `/limo_error_state`：错误状态
- `/limo_emergency_stop`：紧急停止状态

**订阅的话题**：
- `/cmd_vel`：速度命令

**提供的服务**：
- `/emergency_stop`：激活紧急停止
- `/emergency_recovery`：解除紧急停止

## 创新点

1. **自适应运动控制**：根据不同地形和任务自动切换最佳运动模式，提高小车的适应性和灵活性。

2. **智能错误处理**：实现了错误检测和自动处理机制，能够根据错误状态自动调整小车的运行参数，提高系统的稳定性和安全性。

3. **数据可视化**：通过发布TF变换和标准ROS消息，支持使用RViz等工具进行实时数据可视化，方便调试和监控。

4. **安全机制**：实现了多层次安全保护机制，包括紧急停止、自动恢复和基于错误状态的速度限制，确保小车在各种情况下的安全运行。

5. **模块化设计**：每个demo都是独立的功能模块，可以单独使用，也可以组合使用，提高了代码的复用性和灵活性。

## 运行示例

同时运行所有demo的示例：

```bash
# 终端1：启动ROS核心
roscore

# 终端2：启动里程计发布器
rosrun pylimo_demos limo_odometry_publisher.py

# 终端3：启动状态信息发布器
rosrun pylimo_demos limo_status_publisher.py

# 终端4：启动麦轮控制模式切换器
rosrun pylimo_demos limo_mode_controller.py

# 终端5：启动IMU数据发布器
rosrun pylimo_demos limo_imu_publisher.py

# 终端6：启动错误检测与速度控制器
rosrun pylimo_demos limo_error_detection_and_speed_controller.py

# 终端7：启动RViz进行可视化
rosrun rviz rviz
```

## 注意事项

1. 确保串口设备路径正确，默认为`/dev/ttyTHS1`，可能需要根据实际情况调整。

2. 使用前请确保小车电池电量充足，并处于安全环境中。

3. 首次运行时，建议先单独测试每个demo，确保功能正常后再组合使用。

4. 如果遇到权限问题，可以使用以下命令添加串口访问权限：
   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod 666 /dev/ttyTHS1
   ```

5. 如果需要在启动时自动运行这些demo，可以创建launch文件或添加到系统启动脚本中。
