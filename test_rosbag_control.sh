#!/bin/bash

echo "=== RViz插件暂停/播放控制测试 ==="
echo ""

# 检查rosbag进程
echo "1. 检查rosbag进程状态:"
if pgrep -f "rosbag play" > /dev/null; then
    echo "   ✓ rosbag play进程运行中"
    
    # 检查交互模式
    if ps aux | grep "rosbag play" | grep -q -- "-i" || ps aux | grep python | grep "rosbag play -i" > /dev/null; then
        echo "   ✓ 运行在交互模式 (-i)"
    else
        echo "   ✗ 未运行在交互模式，需要 -i 参数"
        echo "   请使用: ./start_interactive_rosbag.sh"
        exit 1
    fi
else
    echo "   ✗ 未检测到rosbag play进程"
    echo "   请先启动: ./start_interactive_rosbag.sh /path/to/your.bag"
    exit 1
fi

echo ""
echo "2. 检查/clock话题:"
if rostopic list | grep -q "/clock"; then
    echo "   ✓ /clock话题存在"
    # 获取时间样本
    time1=$(rostopic echo /clock -n 1 | grep "secs" | awk '{print $2}')
    sleep 0.5
    time2=$(rostopic echo /clock -n 1 | grep "secs" | awk '{print $2}')
    
    if [ "$time1" != "$time2" ]; then
        echo "   ✓ 时间在变化，rosbag正在播放"
        echo "   当前时间: $time2"
    else
        echo "   ! 时间没有变化，rosbag可能已暂停"
        echo "   当前时间: $time2"
    fi
else
    echo "   ✗ /clock话题不存在"
fi

echo ""
echo "3. 测试交互式控制:"
echo "   请在RViz插件中点击'暂停'按钮"
echo "   然后运行此命令验证: rostopic echo /clock -n 2"
echo ""
echo "   或者手动测试:"
echo "   - 在rosbag终端按空格键应该能暂停/恢复播放"
echo "   - 按 'r 2.0' 可以设置2倍速播放"
echo "   - 按 'r 0.5' 可以设置0.5倍速播放"
echo ""

echo "=== 测试完成 ==="
