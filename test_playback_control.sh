#!/bin/bash

echo "=== RViz插件播放控制调试脚本 ==="
echo "这个脚本将测试播放/暂停控制功能"
echo

# 检查rosbag进程
echo "1. 检查rosbag进程状态："
if pgrep -f "rosbag play" > /dev/null; then
    echo "✓ 发现rosbag play进程"
    echo "   进程详情："
    ps aux | grep "rosbag play" | grep -v grep
    
    # 检查是否是交互模式
    if ps aux | grep "rosbag play" | grep -q -- "-i"; then
        echo "✓ rosbag运行在交互模式"
    else
        echo "✗ 警告：rosbag未运行在交互模式！"
        echo "   请使用: rosbag play -i test.bag --clock"
        exit 1
    fi
else
    echo "✗ 未发现rosbag play进程"
    echo "   请先启动: rosbag play -i test.bag --clock"
    exit 1
fi

echo

# 测试命令发送
echo "2. 测试播放控制命令："

# 获取rosbag进程ID
ROSBAG_PID=$(pgrep -f "rosbag play")
echo "   rosbag进程ID: $ROSBAG_PID"

# 检查进程的stdin是否可访问
if [ -w "/proc/$ROSBAG_PID/fd/0" ]; then
    echo "✓ rosbag进程的stdin可访问"
else
    echo "✗ 无法访问rosbag进程的stdin"
    echo "   请确保rosbag运行在交互模式"
    exit 1
fi

echo

# 提供手动测试
echo "3. 手动测试播放控制："
echo "   按任意键发送播放/暂停命令..."
read -n 1 -s

echo "   发送空格键到rosbag进程..."
if echo ' ' > /proc/$ROSBAG_PID/fd/0 2>/dev/null; then
    echo "✓ 命令发送成功"
    echo "   检查rosbag终端是否有反应"
else
    echo "✗ 命令发送失败"
    exit 1
fi

echo

# 速度控制测试
echo "4. 测试速度控制："
echo "   按任意键测试播放速度2x..."
read -n 1 -s

echo "   发送速度命令 'r 2.0'..."
if echo 'r 2.0' > /proc/$ROSBAG_PID/fd/0 2>/dev/null; then
    echo "✓ 速度命令发送成功"
else
    echo "✗ 速度命令发送失败"
fi

echo

echo "5. 重置播放速度到1x："
echo "   发送速度命令 'r 1.0'..."
if echo 'r 1.0' > /proc/$ROSBAG_PID/fd/0 2>/dev/null; then
    echo "✓ 速度重置成功"
else
    echo "✗ 速度重置失败"
fi

echo

echo "=== 测试完成 ==="
echo "如果所有测试都通过，插件应该能够正常控制rosbag播放"
echo "如果有失败，请检查："
echo "1. rosbag是否使用 -i 参数启动"
echo "2. rosbag进程是否仍在运行"
echo "3. 权限是否正确"
echo

# 显示当前rosbag状态
echo "当前rosbag进程详情："
ps aux | grep "rosbag play" | grep -v grep | head -5
