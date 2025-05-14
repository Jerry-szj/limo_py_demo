Header header

# 电池信息
float32 battery_voltage

# 控制模式
uint8 control_mode
uint8 CONTROL_MODE_REMOTE = 0
uint8 CONTROL_MODE_CMD = 1
uint8 CONTROL_MODE_CAN = 2

# 车辆状态
uint8 vehicle_state
uint8 VEHICLE_STATE_NORMAL = 0
uint8 VEHICLE_STATE_ESTOP = 1
uint8 VEHICLE_STATE_EXCEPTION = 2

# 运动模式
uint8 motion_mode
uint8 MOTION_MODE_DIFFERENTIAL = 0
uint8 MOTION_MODE_ACKERMANN = 1
uint8 MOTION_MODE_MECANUM = 2

# 错误代码
uint16 error_code
uint16 ERROR_NONE = 0
uint16 ERROR_BAT_LOW = 1
uint16 ERROR_BAT_CRITICAL = 2
uint16 ERROR_RC_LOST = 4
uint16 ERROR_MOTOR1 = 8
uint16 ERROR_MOTOR2 = 16
uint16 ERROR_MOTOR3 = 32
uint16 ERROR_MOTOR4 = 64
uint16 ERROR_DRIVE_STATUS = 256

# 错误状态描述
string[] error_descriptions
