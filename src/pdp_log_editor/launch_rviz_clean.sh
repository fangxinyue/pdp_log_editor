#!/bin/bash

# 无警告RViz启动脚本 - PDP Log Editor
# 完全抑制TF重复数据警告

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source "$SCRIPT_DIR/../../devel/setup.bash"

# 创建临时的ROS控制台配置文件来抑制TF警告
TEMP_CONFIG=$(mktemp)
cat > "$TEMP_CONFIG" << 'EOF'
# 临时ROS Console配置 - 抑制TF和其他常见警告
log4j.rootLogger=INFO, stdout

# 标准输出appender
log4j.appender.stdout=org.apache.log4j.ConsoleAppender
log4j.appender.stdout.layout=org.apache.log4j.PatternLayout
log4j.appender.stdout.layout.ConversionPattern=[%p] [%d] [%c]: %m%n

# 抑制TF相关的重复数据警告
log4j.logger.ros.tf2_ros=ERROR
log4j.logger.ros.tf=ERROR
log4j.logger.ros.tf2=ERROR
log4j.logger.tf2_ros=ERROR
log4j.logger.tf=ERROR

# 抑制其他常见的无关紧要的警告
log4j.logger.ros.roscpp=ERROR
log4j.logger.ros.rosbag=INFO
log4j.logger.ros.rviz=INFO
EOF

# 设置环境变量
export ROSCONSOLE_CONFIG_FILE="$TEMP_CONFIG"

# 启动函数
launch_rviz() {
    echo "🚀 启动RViz with PDP Log Editor插件 (无TF警告模式)..."
    echo "📁 配置文件: $SCRIPT_DIR/myconfig.rviz"
    echo "🔇 所有TF重复数据警告已抑制"
    echo ""
    echo "📋 使用说明:"
    echo "1. 在Panels菜单中添加 'PDP Log Editor' 面板"
    echo "2. 在Tools菜单中激活 'PDP Log Editor Tool'" 
    echo "3. 使用手动时间捕获或3D点击进行标注"
    echo "4. 点击时间同步按钮切换ROS/Bag时间模式"
    echo ""
    
    # 启动RViz，并将TF警告重定向到/dev/null
    rviz -d "$SCRIPT_DIR/myconfig.rviz" 2> >(grep -v "TF_REPEATED_DATA\|redundant timestamp" >&2)
}

# 清理函数
cleanup() {
    echo ""
    echo "🧹 清理临时文件..."
    rm -f "$TEMP_CONFIG"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 启动RViz
launch_rviz

# 清理
cleanup
