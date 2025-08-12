# PDP Log Editor - RViz Plugin for Driving Scenario Time Annotation

[![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()

## 🎯 项目概述

PDP Log Editor是一个专业的RViz插件，专门为自动驾驶场景时间标注而设计。提供直观的可视化界面和强大的时间管理功能。

### ⭐ 核心特性
- **🎬 智能时间轴**: 视频播放器风格的时间控制界面
- **🎯 精确标注**: 支持4个关键时间点的微秒级标注
- **🤖 自动化工作流**: 引导式操作，简化标注流程  
- **💾 JSON管理**: 完整的配置文件加载和保存
- **🔄 实时同步**: 与rosbag播放完全同步
- **🎨 可视化界面**: 彩色时间标记和拖拽操作

### 📊 功能模块
| 模块 | 功能 | 说明 |
|------|------|------|
| **时间标注** | 4时间戳系统 | start_time → takeover → event → end_time |
| **手动捕获** | 按钮操作 | 2次点击完成核心时间标注 |
| **时间轴控制** | 可视化操作 | 拖拽进度条、调整时间标记 |
| **JSON管理** | 文件I/O | 加载、编辑、保存配置文件 |
| **实时同步** | rosbag集成 | 与/clock话题自动同步 |

---

## 🚀 快速开始

### 📥 获取项目
```bash
cd ~/catkin_ws/src
git clone http://gitlab.zhidaoauto.com/fangxinyue/pdp_log_editor.git
```

### 🔨 编译安装
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 🎮 启动使用
```bash
cd ~/catkin_ws/src/pdp_log_editor
./launch_rviz.sh
```

**🎯 在RViz中**: `Panels` → `Add New Panel` → `pdp_log_editor/PdpLogEditorPanel`

---

## 📚 文档指南

| 文档 | 内容 | 适用对象 |
|------|------|----------|
| **[快速入门指南.md](快速入门指南.md)** | 5分钟上手教程 | 新用户 |
| **[完整用户说明书.md](完整用户说明书.md)** | 详细操作指南和故障排除 | 所有用户 |

---

## 🛠️ 系统要求
- ROS Noetic
- RViz
- Qt5
- nlohmann/json
- rosgraph_msgs

### 安装步骤
1. 克隆仓库到catkin工作空间：
```bash
cd ~/catkin_ws/src
git clone https://gitlab.zhidaoauto.com/fangxinyue/pdp_log_editor.git
```

2. 编译项目：
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 快速启动

### 方法1：使用Launch文件（推荐）
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch pdp_log_editor rviz.launch
```

### 方法2：使用启动脚本
```bash
cd ~/catkin_ws/src/pdp_log_editor
./launch_rviz.sh
```

### 方法3：手动启动
```bash
cd ~/catkin_ws
source devel/setup.bash
rviz -d src/pdp_log_editor/myconfig.rviz
```

## 使用说明

### 插件设置
1. **添加面板**: Panels → Add New Panel → pdp_log_editor/PdpLogEditorPanel
2. **激活工具**: Tools → pdp_log_editor/PdpLogEditorTool

### 时间标注功能
- 使用工具栏按钮加载日志文件
- 右键点击时间轴添加标记
- 使用快捷键导航日志

## 开发指南

### 文件结构
```
pdp_log_editor/
├── CMakeLists.txt
├── package.xml
├── plugin_description.xml
├── src/
│   ├── pdp_log_editor_panel.cpp
│   └── pdp_log_editor_tool.cpp
└── include/
    └── pdp_log_editor/
        ├── pdp_log_editor_panel.h
        └── pdp_log_editor_tool.h
```

### 编译选项
项目使用以下特殊编译选项确保符号可见性：
```cmake
add_compile_options(-fno-lto -fno-fat-lto-objects)
set_target_properties(${PROJECT_NAME} PROPERTIES
  LINK_FLAGS "-Wl,--no-as-needed -Wl,--no-undefined"
  CXX_VISIBILITY_PRESET default
)
```

## 常见问题

### 插件加载失败
如果遇到插件加载错误：
1. 检查是否正确调用了PLUGINLIB_EXPORT_CLASS宏
2. 验证plugin_description.xml中的类名是否匹配
3. 运行以下命令检查符号：
```bash
nm -gDC devel/lib/libpdp_log_editor.so | grep PdpLogEditor
```

### 虚函数表缺失
如果出现`undefined symbol: _ZTVN14pdp_log_editor16PdpLogEditorToolE`错误：
1. 确保所有虚函数都有实现
2. 按照"编译选项"部分设置正确的编译标志

## 贡献指南
欢迎提交Pull Request。请确保：
- 代码符合ROS C++风格指南
- 新功能包含相应的测试用例
- 更新相关文档

## 许可证
本项目采用BSD 3-Clause许可证。