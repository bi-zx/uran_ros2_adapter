# 心跳发送状态广播
# URAN-core 发布当前心跳发送状态

uint64  timestamp_ns     # 时间戳（纳秒）
string  protocol         # 心跳使用的协议
uint64  last_sent_ts     # 上次成功发送时间戳（纳秒）
bool    success          # 最近一次心跳是否发送成功
