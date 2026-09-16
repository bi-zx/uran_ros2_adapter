# 媒体通道控制指令
# URAN-core 下发，URAN-media 订阅

string  action            # "start" | "stop" | "switch" | "record_start" | "record_stop"
string  protocol          # "webrtc" | "rtsp"
string  channel_id        # 指定通道 ID，空串表示全部
string  signal_json       # 信令内容（SDP/ICE 等），JSON 序列化
uint64  timestamp_ns      # 时间戳（纳秒）
