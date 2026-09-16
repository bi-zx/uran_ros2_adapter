# 状态空间字段写入
# 功能包通过此消息向 URAN-core 写入状态空间字段

string  field_name       # 字段名
string  value_json       # JSON 序列化值
bool    persistent       # 是否需要持久化写入
bool    urgent           # 是否紧急（触发即时状态上报）
string  source_pkg       # 来源软件包名
uint64  timestamp_ns     # 写入时间戳（纳秒）
