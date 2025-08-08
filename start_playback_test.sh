#!/bin/bash

echo "=== RViz插件播放控制快速启动脚本 ==="
echo

# 检查是否有test.bag文件
if [ ! -f "test.bag" ]; then
    echo "错误：当前目录没有test.bag文件"
    echo "请将test.bag文件放在当前目录，或修改此脚本中的路径"
    exit 1
fi

# 检查roscore
echo "1. 检查ROS核心..."
if ! pgrep -f "roscore" > /dev/null; then
    echo "   启动roscore..."
    roscore &
    sleep 3
    ROSCORE_PID=$!
    echo "   roscore已启动 (PID: $ROSCORE_PID)"
else
    echo "   roscore已在运行"
fi

echo

# 停止可能存在的旧rosbag进程
echo "2. 清理旧的rosbag进程..."
pkill -f "rosbag play" 2>/dev/null
sleep 1

# 启动rosbag交互模式
echo "3. 启动rosbag交互模式..."
echo "   命令: rosbag play -i test.bag --clock"
rosbag play -i test.bag --clock &
ROSBAG_PID=$!
sleep 2

# 验证rosbag启动
if pgrep -f "rosbag play" > /dev/null; then
    echo "   ✓ rosbag已启动 (PID: $ROSBAG_PID)"
    
    # 检查交互模式
    if ps aux | grep "rosbag play" | grep -q -- "-i"; then
        echo "   ✓ rosbag运行在交互模式"
    else
        echo "   ✗ 警告：rosbag可能未正确启动交互模式"
    fi
else
    echo "   ✗ rosbag启动失败"
    exit 1
fi

echo

# 等待用户启动RViz
echo "4. 现在可以启动RViz了："
echo "   在新终端中运行: rviz"
echo "   然后添加PdpLogEditorPanel插件"
echo

echo "5. 按Ctrl+C停止所有进程"
echo

# 等待用户中断
trap 'echo ""; echo "清理进程..."; pkill -f "rosbag play"; echo "rosbag已停止"; exit 0' INT

echo "rosbag正在运行中...按Ctrl+C停止"
wait $ROSBAG_PID
