#!/bin/bash

# RViz启动脚本 - PDP Log Editor插件
# 使用项目自带的配置文件启动RViz

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source "$SCRIPT_DIR/../../devel/setup.bash"

# 启动RViz并加载配置文件
echo "🚀 启动RViz with PDP Log Editor插件..."
echo "📁 配置文件: $SCRIPT_DIR/myconfig.rviz"
echo ""
echo "📋 使用说明:"
echo "1. 在Panels菜单中添加 'PDP Log Editor' 面板"
echo "2. 在Tools菜单中激活 'PDP Log Editor Tool'"
echo "3. 使用手动时间捕获或3D点击进行标注"
echo "4. 点击时间同步按钮切换ROS/Bag时间模式"
echo ""

rviz -d "$SCRIPT_DIR/myconfig.rviz"
