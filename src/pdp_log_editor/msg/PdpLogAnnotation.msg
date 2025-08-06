# PdpLogAnnotation.msg
# 描述一个完整的pdp logsim case时序标注结果

std_msgs/Header header

# 关联的源日志文件名
string source_log_name

# 四个关键时间戳
time scene_start_time
time takeover_time
time event_time
time scene_end_time