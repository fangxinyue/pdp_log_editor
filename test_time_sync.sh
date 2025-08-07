#!/bin/bash

echo "=== 时间同步测试脚本 ==="
echo ""

# 检查roscore是否运行
if ! pgrep -f "roscore" > /dev/null; then
    echo "启动 roscore..."
    gnome-terminal -- bash -c "cd /home/fang/catkin_ws && source devel/setup.bash && roscore; exec bash"
    sleep 3
fi

echo "1. 启动 rosbag play（带 --clock 参数）"
echo "   这将发布 /clock 话题，提供 bag 时间"
gnome-terminal -- bash -c "cd /home/fang/catkin_ws && source devel/setup.bash && rosbag play test.bag --clock -l; exec bash"

sleep 2

echo ""
echo "2. 启动 RViz"
echo "   请在 RViz 中添加 PDP Log Editor Panel 插件"
gnome-terminal -- bash -c "cd /home/fang/catkin_ws && source devel/setup.bash && rviz; exec bash"

echo ""
echo "=== 测试说明 ==="
echo "1. 在 RViz 中添加 'PDP Log Editor Panel' 插件"
echo "2. 现在手动获取时间使用的是 bag 时间（来自 /clock 话题）"
echo "3. 时间轴显示的也是 bag 时间"
echo "4. 两者现在应该是一致的"
echo ""
echo "=== 主要修复 ==="
echo "- 手动获取时间现在使用 current_timeline_time_（bag时间）"
echo "- 移除了时间模式切换，统一使用 bag 时间"
echo "- 界面显示更清楚地标明使用 'Bag时间'"
echo "- 状态按钮显示当前 bag 时间值"
echo ""
echo "按任意键关闭..."
read -n 1
