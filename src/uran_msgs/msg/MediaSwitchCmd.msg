# 流媒体通道切换通知
# URAN-core 发布，URAN-media 订阅

string  action             # "start" | "stop" | "switch"
string  protocol           # "webrtc" | "rtsp"
uint64  timestamp_ns       # 时间戳（纳秒）
