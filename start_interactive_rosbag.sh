#!/bin/bash

# 启动交互模式rosbag播放的脚本
# 用法: ./start_interactive_rosbag.sh [bag文件路径]

BAG_FILE=${1:-"test.bag"}

echo "停止现有的rosbag进程..."
pkill -f "rosbag play" 2>/dev/null || true
sleep 1

echo "启动交互模式rosbag播放: $BAG_FILE"
echo "使用空格键暂停/恢复播放"
echo "使用 's <time>' 跳转到指定时间"
echo "使用 'r <rate>' 设置播放速度"
echo "使用 'q' 退出"
echo ""

# 启动交互模式rosbag (-i 是交互模式的关键参数)
rosbag play -i --clock --loop --rate 1.0 "$BAG_FILE"
