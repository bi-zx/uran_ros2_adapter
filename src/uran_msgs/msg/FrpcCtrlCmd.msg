# 端口转发控制指令
# URAN-core 下发，URAN-frpcpoint 订阅

string  action              # "start" | "stop" | "update"
string  frps_host           # frp 服务端地址
uint32  frps_port           # frp 服务端端口
string  service_name        # 服务标识（如 "ssh"）
uint32  local_port          # 本机服务端口（如 22）
uint32  remote_port         # 服务端分配的远程端口
string  auth_token          # frp 鉴权 token
uint64  timestamp_ns        # 时间戳（纳秒）
