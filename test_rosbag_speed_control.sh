#!/bin/bash

echo "=== Rosbag速度控制测试脚本 ==="

# 检查rosbag进程是否存在
if pgrep -f 'rosbag play' > /dev/null; then
    echo "✓ 检测到rosbag进程运行中"
    
    # 检查是否是交互模式
    if ps aux | grep 'rosbag play' | grep -q -- '-i'; then
        echo "✓ Rosbag运行在交互模式 (-i)"
        echo "📝 测试速度控制命令..."
        
        # 测试不同的速度设置方法
        echo "方法1: 直接写入stdin"
        if echo 'r 0.5' > /proc/$(pgrep -f 'rosbag play')/fd/0 2>/dev/null; then
            echo "✓ 成功发送速度命令 r 0.5"
            sleep 2
            echo 'r 1.0' > /proc/$(pgrep -f 'rosbag play')/fd/0 2>/dev/null
            echo "✓ 恢复正常速度 r 1.0"
        else
            echo "✗ 方法1失败"
            
            echo "方法2: 使用管道"
            if echo 'r 0.5' | cat > /proc/$(pgrep -f 'rosbag play')/fd/0 2>/dev/null; then
                echo "✓ 方法2成功"
                sleep 2
                echo 'r 1.0' | cat > /proc/$(pgrep -f 'rosbag play')/fd/0 2>/dev/null
                echo "✓ 恢复正常速度"
            else
                echo "✗ 方法2也失败"
            fi
        fi
        
        echo ""
        echo "🎮 交互式命令说明:"
        echo "  空格键: 暂停/恢复播放"
        echo "  r <速度>: 设置播放速度 (例如: r 0.5, r 2.0)"
        echo "  s <时间>: 跳转到指定时间"
        echo "  q: 退出播放"
        
    else
        echo "⚠️  Rosbag运行在普通模式，无法交互控制"
        echo "❌ 需要使用 'rosbag play -i test.bag --clock' 启动交互模式"
    fi
else
    echo "❌ 未检测到rosbag进程"
    echo "💡 请先启动rosbag: rosbag play -i test.bag --clock"
fi

echo ""
echo "=== 测试完成 ==="
