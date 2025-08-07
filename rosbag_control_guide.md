# RViz PDP Log Editor - Rosbag 控制指南

## 新增功能

### 1. 真实的 Rosbag 控制
现在插件可以真正控制通过命令行运行的 `rosbag play test.bag --clock`。

### 2. 控制方法
插件使用多种方法确保控制的可靠性：

#### A. 直接进程控制（最可靠）
- 通过 `/proc` 文件系统直接向 rosbag 进程发送键盘命令
- 空格键：播放/暂停切换
- 's <时间>'：跳转到指定时间
- 'r <速率>'：设置播放速率

#### B. ROS 话题控制（备用方案）
- `/rosbag/control`：通用控制命令
- `/rosbag/pause`：暂停控制
- `/rosbag/seek`：时间跳转

### 3. 使用步骤

#### 步骤 1：启动 rosbag
```bash
cd /home/fang/catkin_ws
rosbag play test.bag --clock
```

#### 步骤 2：启动 RViz
```bash
# 在另一个终端
cd /home/fang/catkin_ws
source devel/setup.bash
rosrun rviz rviz
```

#### 步骤 3：添加插件
1. 在 RViz 中，点击 "Panels" -> "Add New Panel"
2. 选择 "pdp_log_editor/PdpLogEditorPanel"

#### 步骤 4：测试控制功能

##### 播放/暂停控制
- 点击绿色"播放"按钮 → 变为橙色"暂停"，rosbag 开始播放
- 点击橙色"暂停"按钮 → 变为绿色"播放"，rosbag 暂停播放
- 红色"停止"按钮 → 完全停止 rosbag

##### 时间轴拖拽控制
- 拖拽时间轴滑动条到任意位置
- Rosbag 会跳转到对应时间点
- 状态栏显示"跳转到时间: XXXs"

##### 播放速度控制
- 调整速度输入框（0.1x 到 5.0x）
- Rosbag 播放速度会相应调整
- 状态栏显示"播放速度设置为: XXXx"

### 4. 实时时间同步
- 插件会自动订阅 `/clock` 话题
- 时间轴会实时显示当前 bag 时间
- 时间范围会自动调整以适应 bag 内容

### 5. 故障排除

#### 如果控制不起作用：
1. 确认 rosbag 以 `--clock` 参数运行
2. 检查 rosbag 进程是否存在：
   ```bash
   pgrep -f "rosbag play"
   ```
3. 确认 `/clock` 话题正在发布：
   ```bash
   rostopic hz /clock
   ```

#### 调试信息：
- 查看 RViz 控制台输出（rosout）
- 状态栏会显示控制命令执行状态

### 6. 键盘快捷键对照
在 rosbag 交互模式中：
- 空格：播放/暂停
- s：跳转到时间
- r：设置播放速率
- q：退出

插件会自动发送这些命令到 rosbag 进程。

### 7. 兼容性
- 支持 ROS Noetic
- 兼容标准 rosbag 格式
- 支持带 `--clock` 参数的 bag 播放

## 注意事项
1. 必须先启动 rosbag，再启动 RViz 插件
2. 时间跳转可能需要几秒钟才能生效（取决于 bag 大小）
3. 播放速度调整在某些 bag 文件上可能有延迟
