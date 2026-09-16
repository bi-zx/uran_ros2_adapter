# 统一上行数据载体
# 功能包通过此消息向 URAN-core 上报数据

string  source_pkg          # 来源软件包
string  data_type           # 数据类型："sensor"/"state"/"event"/"media_signal" 等
string  preferred_protocol  # 偏好上行协议，空串表示跟随当前主协议
string  payload_json        # JSON 序列化的数据体
bool    urgent              # 是否紧急
uint64  timestamp_ns        # 时间戳（纳秒）
