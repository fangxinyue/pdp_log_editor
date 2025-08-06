# PDP Log Editor - Rviz Plugin

## 项目概述
PDP Log Editor是一个Rviz插件，提供日志编辑和可视化功能。主要功能包括：
- 日志标记和注释
- 时间轴导航
- 自定义可视化工具

## 安装指南

### 依赖项
- ROS Noetic
- Rviz
- Qt5
- Boost

### 安装步骤
1. 克隆仓库到catkin工作空间：
```bash
cd ~/catkin_ws/src
git clone https://github.com/your-repo/pdp_log_editor.git
```

2. 安装依赖：
```bash
rosdep install --from-paths . --ignore-src -y
```

3. 编译项目：
```bash
cd ~/catkin_ws
catkin build pdp_log_editor
source devel/setup.bash
```

## 使用说明

### 启动插件
1. 启动Rviz：
```bash
rosrun rviz rviz
```

2. 添加PDP Log Editor面板：
- 点击菜单栏的"Panels" > "Add New Panel"
- 选择"pdp_log_editor/PdpLogEditorPanel"

### 基本操作
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