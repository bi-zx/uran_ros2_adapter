# 状态空间全量快照广播
# URAN-core 定期广播当前状态空间快照

string  msg_version      # 消息版本
uint64  timestamp_ns     # 时间戳（纳秒）
string  device_id        # 装备实例 ID
string  fields_json      # 全量字段的 JSON 序列化
