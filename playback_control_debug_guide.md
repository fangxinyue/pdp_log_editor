# RViz插件播放控制测试指南

## 问题诊断
你的插件按钮不响应的问题可能有以下原因：

## 1. 启动顺序检查

**正确的启动顺序：**
```bash
# 步骤1：确保roscore运行
roscore

# 步骤2：启动rosbag（必须使用-i交互模式）
rosbag play -i test.bag --clock

# 步骤3：启动RViz并加载插件
rviz
```

## 2. 关键问题排查

### 检查rosbag是否运行在交互模式：
```bash
ps aux | grep "rosbag play"
```
输出中必须包含 `-i` 参数，如：
```
user 12345 ... rosbag play -i test.bag --clock
```

### 如果没有 `-i` 参数，请重新启动rosbag：
```bash
# 停止当前rosbag
pkill -f "rosbag play"

# 重新启动（注意-i参数）
rosbag play -i test.bag --clock
```

## 3. 手动测试播放控制

运行我们的调试脚本：
```bash
cd /home/fang/catkin_ws
./test_playback_control.sh
```

这个脚本会：
- 检查rosbag进程状态
- 验证交互模式
- 测试命令发送
- 提供手动测试机会

## 4. 在RViz中测试

1. **加载插件：**
   - 启动rviz
   - Panels → Add New Panel → PdpLogEditorPanel

2. **测试按钮：**
   - 点击"播放/暂停"按钮
   - 观察状态文本变化
   - 检查rosbag终端是否有反应

3. **查看控制台输出：**
   ```bash
   # 在启动rviz的终端查看日志
   # 应该看到类似信息：
   # [INFO]: Resumed rosbag playback
   # [INFO]: Paused rosbag playback
   ```

## 5. 常见问题解决

### 问题1：按钮点击无反应
**解决方案：**
- 确保rosbag使用 `-i` 参数启动
- 检查插件是否正确加载
- 查看RViz控制台错误信息

### 问题2：错误提示"未检测到rosbag进程"
**解决方案：**
```bash
# 检查rosbag是否运行
pgrep -f "rosbag play"

# 如果没有输出，启动rosbag
rosbag play -i test.bag --clock
```

### 问题3：错误提示"rosbag未运行在交互模式"
**解决方案：**
```bash
# 停止当前rosbag
pkill -f "rosbag play"

# 重新启动时加上-i参数
rosbag play -i test.bag --clock
```

## 6. 验证插件工作

**成功标志：**
1. 按钮点击后文本改变（播放↔暂停）
2. 按钮颜色改变（绿色↔橙色）
3. 状态标签显示相应信息
4. rosbag终端显示暂停/恢复消息

**rosbag交互模式命令：**
- `空格键`：播放/暂停切换
- `r 2.0`：设置播放速度为2倍
- `r 1.0`：恢复正常播放速度
- `s`：单步播放

## 7. 完整测试流程

```bash
# 1. 启动roscore（如果还没启动）
roscore &

# 2. 启动交互模式rosbag
rosbag play -i test.bag --clock &

# 3. 运行测试脚本
./test_playback_control.sh

# 4. 启动RViz并测试插件
rviz
```

## 注意事项
- **必须使用 `-i` 参数启动rosbag**
- 插件需要rosbag进程在运行时才能工作
- 如果更改了代码，需要重新编译并重启RViz
- 确保有test.bag文件用于测试
