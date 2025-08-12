#!/bin/bash

# RViz配置清理脚本 - 移除有问题的插件引用

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/myconfig.rviz"
BACKUP_FILE="$SCRIPT_DIR/myconfig.rviz.backup"

echo "🧹 清理RViz配置文件..."

# 创建备份
if [ -f "$CONFIG_FILE" ]; then
    cp "$CONFIG_FILE" "$BACKUP_FILE"
    echo "✅ 已创建配置文件备份: myconfig.rviz.backup"
else
    echo "❌ 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

# 移除有问题的插件引用
if grep -q "rviz_plugin_tutorials/Imu" "$CONFIG_FILE"; then
    echo "🔧 移除有问题的Imu插件引用..."
    sed -i '/rviz_plugin_tutorials\/Imu/d' "$CONFIG_FILE"
    echo "✅ 已移除Imu插件引用"
else
    echo "ℹ️  未发现Imu插件引用"
fi

# 检查并修复其他常见问题
echo "🔧 检查其他插件问题..."

# 移除不存在的插件类
sed -i '/Class:.*rviz_plugin_tutorials/d' "$CONFIG_FILE"

echo "✅ 配置文件清理完成！"
echo "📁 原始文件备份: myconfig.rviz.backup"
echo "📁 清理后文件: myconfig.rviz"
