# PDP日志编辑器 - 用户说明书

## 概述

PDP日志编辑器是一个ROS RViz面板插件，用于标注和编辑自动驾驶场景的关键时间点。该工具支持JSON格式的配置文件加载和保存，并提供直观的时间轴界面进行时间戳管理。

## 系统要求

- ROS Noetic
- Qt5开发环境
- RViz
- nlohmann/json库
- 支持rosbag播放功能

## 安装与编译

### 1. 环境准备
```bash
# 确保已安装ROS Noetic
source /opt/ros/noetic/setup.bash

# 进入catkin工作空间
cd /home/fang/catkin_ws

# 编译项目
catkin_make

# 设置环境变量
source devel/setup.bash
```

### 2. 启动RViz并加载插件
```bash
# 启动RViz
roscore &
rviz
```

在RViz中：
1. 点击菜单 `Panels` → `Add New Panel`
2. 选择 `pdp_log_editor/PdpLogEditorPanel`
3. 面板将出现在RViz界面中

## 界面介绍

### 主要组件

1. **状态提示区域** - 显示当前操作状态和提示信息
2. **时间参数编辑区** - 四个关键时间点的数值输入框
3. **控制按钮区** - 各种操作按钮
4. **时间轴可视化区** - 图形化时间轴显示
5. **JSON路径显示** - 当前加载的配置文件路径

### 时间参数说明

| 参数名称 | 颜色标识 | 含义 | 用途 |
|---------|---------|------|------|
| start_time | 浅绿色 | 场景开始时间 | 标记场景起始点 |
| takeover | 浅黄色 | 控车时间 | 人工接管时间点 |
| event | 浅橙色 | 事件时间 | 关键事件发生时间 |
| end_time | 浅红色 | 场景结束时间 | 标记场景结束点 |
| vehicle_config | 浅灰色 | 车辆配置 | 车辆标识信息 |

## 操作流程

### 标准操作流程

#### 第一步：加载JSON配置
1. **初始状态**：启动后只有"Load from JSON"按钮可用
2. **点击操作**：点击 `Load from JSON` 按钮
3. **文件选择**：在弹出的文件对话框中选择JSON配置文件
4. **自动设置**：
   - 系统自动加载时间参数
   - 时间轴自动设置为合适范围
   - 所有按钮和编辑框变为可用状态

#### 第二步：手动时间捕获（可选）
1. **启动捕获**：点击 `手动获取时间 (0/2)` 按钮
2. **第一次点击**：捕获takeover（控车）时间
   - 状态显示：`Captured takeover: XXXs (时间源)`
   - 按钮变为：`手动获取时间 (1/2)`
3. **第二次点击**：捕获event（事件）时间
   - 状态显示：`已捕获事件时间: XXXs (时间源)`
   - 按钮变为：`手动获取完成 (2/2)`

#### 第三步：时间调整（可选）
通过以下方式精确调整时间：
1. **直接编辑**：在数值输入框中手动输入精确时间
2. **时间轴操作**：
   - 拖动进度条跳转到指定时间
   - 点击时间轴上的时间戳标记进行微调
   - 拖拽时间戳标记到新位置

#### 第四步：保存配置
1. **点击保存**：点击 `Save to JSON` 按钮
2. **选择路径**：选择保存位置和文件名
3. **确认保存**：系统保存完整的JSON配置文件

### 高级操作

#### 撤销操作
- **撤销手动捕获**：撤销最近一次手动时间捕获
- **全部重置**：清空所有手动捕获的时间，保留start_time和end_time

#### 时间轴控制
1. **时间跳转**：拖动进度条可控制rosbag播放进度
2. **暂停/播放**：通过ROS参数 `/sim_pause` 控制播放状态
3. **实时同步**：自动与 `/clock` 话题同步显示当前时间

## 详细功能说明

### JSON配置文件格式

```json
{
  "case_time": {
    "start_time": 1234567890.123456,
    "takeover": 1234567895.234567,
    "event": 1234567900.345678,
    "end_time": 1234567905.456789
  },
  "vehicle_config": "JLBJA47708D",
  "case_info": {
    "ego_initial": {
      "position": {"x": -4164.322722157463, "y": 0.0, "z": 488.3309675815399},
      "rotation": {"x": 0.0, "y": 250.71370161918355, "z": 0.0}
    },
    "ego_dest": {"x": -4229.899380000308, "y": 0.0, "z": 183.6990550000337},
    "scene": "environment_GuoZhan"
  },
  "db.sqlite": "https://autocar-mogosim-1255510688.cos.ap-beijing.myqcloud.com/hadmap/bj-V2.9.8.sqlite"
}
```

### 时间源识别

系统自动识别时间来源：
- **ROS系统时间**：使用系统当前时间
- **Rosbag仿真时间**：从 `/clock` 话题获取的仿真时间

### Rosbag控制功能

#### 前提条件
确保rosbag以交互模式播放：
```bash
# 正确的启动方式
rosbag play --clock --pause your_bag_file.bag

# 或者
rosbag play --clock your_bag_file.bag
```

#### 支持的控制命令
- **时间跳转**：拖动时间轴进度条
- **暂停/播放切换**：通过 `/sim_pause` 参数
- **实时同步**：自动跟踪当前播放时间

## 快捷操作指南

### 基本工作流程
```
启动RViz → 加载面板 → Load JSON → 调整时间 → Save JSON
```

### 快速时间捕获
```
Load JSON → 手动获取时间(第1次) → 手动获取时间(第2次) → Save JSON
```

### 仅编辑已有配置
```
Load JSON → 编辑数值框 → Save JSON
```

## 故障排除

### 常见问题

#### 1. 时间跳转不工作
**症状**：拖动时间轴进度条无反应
**解决方案**：
- 检查rosbag是否正在运行：`pgrep -f 'rosbag play'`
- 确保rosbag以交互模式启动
- 重新启动rosbag播放

#### 2. 无法捕获仿真时间
**症状**：只能获取系统时间，无法获取rosbag时间
**解决方案**：
- 确认 `/clock` 话题存在：`rostopic list | grep clock`
- 检查rosbag是否使用 `--clock` 参数启动
- 验证时间发布：`rostopic echo /clock`

#### 3. JSON加载失败
**症状**：加载JSON文件时报错
**解决方案**：
- 检查JSON文件格式是否正确
- 验证文件权限
- 检查时间字段是否为有效数值

#### 4. 插件无法加载
**症状**：RViz中找不到PdpLogEditorPanel
**解决方案**：
- 重新编译：`catkin_make`
- 刷新环境：`source devel/setup.bash`
- 检查编译错误信息

### 调试命令

```bash
# 检查ROS环境
env | grep ROS

# 查看可用话题
rostopic list

# 监控时钟话题
rostopic echo /clock

# 检查rosbag进程
ps aux | grep rosbag

# 查看RViz日志
rosrun rqt_console rqt_console
```

## 最佳实践

### 1. 工作流程建议
- 始终先加载JSON配置，再进行其他操作
- 使用手动捕获功能时，确保rosbag正在播放
- 定期保存工作进度，避免数据丢失

### 2. 时间精度注意事项
- 时间戳精度支持到微秒级别（6位小数）
- 手动编辑时间时注意数值范围和有效性
- 时间轴操作提供直观的可视化反馈

### 3. 文件管理
- 使用有意义的文件名和路径
- 保持JSON配置文件的备份
- 定期整理和归档历史配置

### 4. 性能优化
- 避免在大型rosbag文件上频繁跳转
- 合理设置时间轴范围，避免过大的时间跨度
- 监控系统资源使用情况

## 技术支持

### 系统要求检查
```bash
# 检查ROS版本
rosversion -d

# 检查Qt版本
qmake --version

# 检查编译环境
catkin_make --version
```

### 日志收集
如需技术支持，请提供：
1. RViz控制台输出
2. catkin_make编译日志
3. 具体的错误截图
4. 使用的JSON配置文件示例

---

**版本信息**：v1.0  
**最后更新**：2025年8月12日  
**支持系统**：Ubuntu 20.04 + ROS Noetic
