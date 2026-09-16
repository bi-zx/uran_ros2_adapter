# 统一运控指令
# 坐标系：ROS REP-103 右手体坐标系（x前/y左/z上，yaw逆时针为正）

string  msg_version             # 消息版本
string  device_id               # 设备 ID
uint64  timestamp_ns            # 时间戳（纳秒）
string  controller              # 指令来源："cloud" / "field" / "auto"

# 运动控制
float64 linear_vel_x            # 前进速度（m/s）：正=前，负=后
float64 linear_vel_y            # 横向速度（m/s）：正=左，负=右
float64 linear_vel_z            # 垂直速度（m/s）：正=上，负=下
float64 angular_vel_z           # 偏航角速度（rad/s）：正=左转（逆时针）

# 姿态控制（可选，设备不支持时忽略）
float64 target_roll             # 目标横滚角（rad）
float64 target_pitch            # 目标俯仰角（rad）
float64 target_yaw              # 目标偏航角（rad），NaN 表示不指定

# 特殊动作
string  action                  # "stop"/"emergency_stop"/"takeoff"/"land"/"return_home"/""

# 扩展字段
string  extra_json              # JSON 序列化的扩展参数
