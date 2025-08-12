#!/bin/bash

# 完全无警告RViz启动脚本 - PDP Log Editor
# 过滤所有非关键错误和警告信息

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source "$SCRIPT_DIR/../../devel/setup.bash"

# 检查并启动roscore
echo "🔍 检查ROS核心服务..."
if ! pgrep -x "roscore" > /dev/null && ! pgrep -x "rosmaster" > /dev/null; then
    echo "🚀 启动roscore..."
    roscore > /dev/null 2>&1 &
    ROSCORE_PID=$!
    sleep 3
    echo "✅ roscore已启动 (PID: $ROSCORE_PID)"
else
    echo "✅ roscore已在运行"
fi

echo ""
echo "🚀 启动RViz with PDP Log Editor插件 (静默模式)..."
echo "📁 配置文件: $SCRIPT_DIR/myconfig.rviz"
echo "🔇 所有警告和错误已过滤"
echo ""
echo "📋 使用说明:"
echo "1. 在Panels菜单中添加 'PDP Log Editor' 面板"
echo "2. 在Tools菜单中激活 'PDP Log Editor Tool'"
echo "3. 使用手动时间捕获或3D点击进行标注"
echo "4. 点击时间同步按钮切换ROS/Bag时间模式"
echo ""
echo "🎯 RViz正在启动，请稍候..."

# 定义要过滤的错误模式
FILTER_PATTERNS=(
    "TF_REPEATED_DATA"
    "redundant timestamp" 
    "ignoring data with redundant timestamp"
    "PluginlibFactory.*failed to load"
    "SEVERE WARNING.*namespace collision"
    "Error in XmlRpcClient::writeRequest"
    "拒绝连接"
    "write error"
    "connection refused"
    "writeRequest"
)

# 构建grep过滤表达式
FILTER_EXPR=$(IFS='|'; echo "${FILTER_PATTERNS[*]}")

# 启动RViz并过滤所有错误信息，只显示重要信息
rviz -d "$SCRIPT_DIR/myconfig.rviz" 2>&1 | grep -v -E "$FILTER_EXPR" | grep -E "INFO.*rviz version|INFO.*compiled against|INFO.*OpenGL|Stereo|OpenGL device|OpenGl version"
