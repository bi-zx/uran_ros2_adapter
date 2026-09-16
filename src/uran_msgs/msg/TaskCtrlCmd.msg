# 任务控制指令
# URAN-core 下发，URAN-autotask 订阅

string  msg_version         # 消息版本
string  task_id             # 任务唯一 ID
string  action              # "start" | "pause" | "resume" | "stop" | "update_params"
string  task_type           # "waypoint" | "patrol" | "custom"
string  task_params_json    # 任务参数（JSON 序列化）
uint64  timestamp_ns        # 时间戳（纳秒）
