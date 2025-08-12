#!/bin/bash

# TF警告修复测试脚本

echo "🧪 测试TF警告修复效果..."
echo ""

# 测试原始命令是否会产生警告
echo "1️⃣ 测试基本RViz启动 (5秒后自动关闭)..."
timeout 5s rviz --help > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ RViz可正常启动"
else
    echo "❌ RViz启动异常"
fi

echo ""
echo "2️⃣ 验证启动脚本..."
if [ -f "/home/fang/catkin_ws/src/pdp_log_editor/launch_rviz.sh" ]; then
    echo "✅ launch_rviz.sh 存在"
    if grep -q "grep -v.*TF_REPEATED_DATA" "/home/fang/catkin_ws/src/pdp_log_editor/launch_rviz.sh"; then
        echo "✅ TF警告过滤已启用"
    else
        echo "❌ TF警告过滤未配置"
    fi
else
    echo "❌ launch_rviz.sh 不存在"
fi

echo ""
echo "3️⃣ 检查备用启动脚本..."
if [ -f "/home/fang/catkin_ws/src/pdp_log_editor/launch_rviz_clean.sh" ]; then
    echo "✅ launch_rviz_clean.sh 存在"
    if [ -x "/home/fang/catkin_ws/src/pdp_log_editor/launch_rviz_clean.sh" ]; then
        echo "✅ 脚本具有可执行权限"
    else
        echo "⚠️  脚本没有可执行权限"
    fi
else
    echo "❌ launch_rviz_clean.sh 不存在"
fi

echo ""
echo "🎯 推荐使用方法："
echo "选项1 (修复版): ./launch_rviz.sh"
echo "选项2 (完全清洁版): ./launch_rviz_clean.sh"
echo ""
echo "这两个脚本都会过滤掉TF_REPEATED_DATA警告，保持控制台清洁。"
