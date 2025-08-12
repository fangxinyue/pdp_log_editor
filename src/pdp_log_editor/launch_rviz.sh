#!/bin/bash

# RViz启动脚本 - PDP Log Editor插件 (抑制TF警告版本)
# 使用项目自带的配置文件启动RViz

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source "$SCRIPT_DIR/../../devel/setup.bash"

# 启动RViz并加载配置文件
# 检查并启动roscore
echo "🔍 检查ROS核心服务..."
if ! pgrep -x "roscore" > /dev/null && ! pgrep -x "rosmaster" > /dev/null; then
    echo "🚀 启动roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 3
    echo "✅ roscore已启动 (PID: $ROSCORE_PID)"
else
    echo "✅ roscore已在运行"
fi

echo ""
echo "🚀 启动RViz with PDP Log Editor插件..."
echo "📁 配置文件: $SCRIPT_DIR/myconfig.rviz"
echo "🔇 错误信息和警告已过滤"
echo ""
echo "📋 使用说明:"
echo "1. 在Panels菜单中添加 'PDP Log Editor' 面板"
echo "2. 在Tools菜单中激活 'PDP Log Editor Tool'"
echo "3. 使用手动时间捕获或3D点击进行标注"
echo "4. 点击时间同步按钮切换ROS/Bag时间模式"
echo ""

# 启动RViz并过滤所有常见的错误和警告信息
rviz -d "$SCRIPT_DIR/myconfig.rviz" 2>&1 | grep -v -E "TF_REPEATED_DATA|redundant timestamp|ignoring data with redundant timestamp|PluginlibFactory.*failed to load|SEVERE WARNING.*namespace collision|Error in XmlRpcClient::writeRequest|拒绝连接|write error"
