# PDP Log Editor - RViz Plugin for Driving Scenario Time Annotation

## 项目概述
PDP Log Editor是一个RViz插件，专门用于驾驶场景的时间标注。主要功能包括：
- **4时间戳标注系统**: start_time → takeover → event → end_time
- **手动时间捕获**: 按钮式时间戳获取
- **3D场景交互**: 鼠标点击进行标注
- **时间同步功能**: 支持ROS时间和Bag播放时间切换
- **JSON文件I/O**: 保存和加载标注数据

## 安装指南

### 依赖项
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