1.  项目概述

PDPC logsim场景编辑插件是一个基于 ROS (Robot Operating System) 和 RViz 的面板插件，专为自动驾驶场景数据的高效标注而设计。它通过直观的图形界面，帮助用户管理和编辑测试场景中的关键时间点，并将结果保存为标准化的 JSON 配置文件。

核心功能包括：

    时间轴可视化：直观展示场景的完整时间线。

    交互式时间标注：支持手动捕获、数值输入和拖拽等多种标注方式。

    JSON 配置管理：以 JSON 文件为核心，实现标注数据的加载、编辑和保存。


2. 步骤一：下载并解压

2.1 下载

在项目的 GitLab 页面上，下载名为 pdp_log_editor_release.tar.gz 的文件。
2.2  解压

为了方便管理，建议在主目录下创建一个专门存放 RViz 插件的文件夹，例如 ~/rviz_plugins。

首先，在终端中执行以下命令创建并进入该目录：

mkdir -p ~/rviz_plugins && cd ~/rviz_plugins

然后，使用 tar 命令解压您下载的压缩包（此处假设您把下载的文件放在了 ~/Downloads 文件夹）：

tar -xzvf ~/下载/pdp_log_editor_release.tar.gz

.
├── launch_rviz.sh         <-- 启动脚本在顶层，方便执行
└── pdp_log_editor/        <-- 插件核心包
    ├── lib/
    │   └── libpdp_log_editor.so
    ├── myconfig.rviz
    ├── package.xml
    └── plugin_description.xml



3. 步骤二：启动与加载插件

[方新悦 > PDPC logsim场景编辑 RViz 插件用户说明书 > image2025-8-11_14-50-41.png] [方新悦 > PDPC logsim场景编辑 RViz 插件用户说明书 > image2025-8-15_11-59-23.png] [方新悦 > PDPC logsim场景编辑 RViz 插件用户说明书 > image2025-8-11_14-48-2.png]
3.1 第一个终端启动roscore

roscore

3.2 第二个终端启动脚本 

项目提供了一个一键启动脚本，可自动处理环境配置并加载 RViz。

1. 进入项目目录

cd ~/rviz_plugins


2. 授予脚本执行权限 (仅需一次)
chmod +x launch_rviz.sh

3. 运行启动脚本
./launch_rviz.sh


[方新悦 > PDPC logsim场景编辑 RViz 插件用户说明书 > 2025-08-12 17-06-38 的屏幕截图.png]
另：手动启动方式

您也可以手动启动 RViz。加载环境并使用预设配置启动RViz

source ~/catkin_ws/devel/setup.
rviz -d ~/catkin_ws/src/pdp_log_editor/myconfig.rviz

插件加载后，将出现在 RViz 界面右侧。如果没有自动出现，请通过菜单 Panels → Add New Panel → pdp_log_editor/PdpLogEditorPanel 手动添加。
3.3 第三个终端播放Rosbag

rosbag play test.bag --clock -l

        以2倍速播放:

        rosbag play test.bag --clock -l -r 2

        以一半（0.5倍）的速度慢放:

        rosbag play test.bag --clock -l -r 0.5

        按空格暂停，Ctrl+Z结束播放

3.4 核心操作：时间标注

[方新悦 > PDPC logsim场景编辑 RViz 插件用户说明书 > image2025-8-13_10-20-41.png]

演示视频：

加载配置：在插件面板中，点击 Load from JSON 按钮，选择一个已有的配置文件。加载成功后，所有功能按钮将被激活。

您有以下三种方式来标注或调整时间：

方式
	

操作说明
	

适用场景

方式1：一键捕获
	

在 rosbag 播放时，依次点击 手动获取时间 (0/2) 按钮两次，即可快速捕获 takeover (接管) 和 event (事件) 的时间点。
	

快速标注正在实时回放的场景。

方式2：数值编辑
	

在面板上对应颜色的输入框中，直接填写精确的时间戳数值。
	

需要高精度微调或已知确切时间值。

方式3：时间轴交互
	

在下方时间轴上，直接拖动代表各个时间点的彩色菱形标记，或拖动主进度条进行跳转。
	

直观地进行粗略定位和快速调整。
3.5 完成工作：保存结果

成所有标注和调整后，点击 Save to JSON 按钮，选择路径并保存文件。系统会将所有更新写入 JSON 文件，并保持所有功能可用，以便您继续编辑或加载下一个任务。
3.6 高级功能

    撤销/重置：撤销上一步可取消最近一次的手动捕获；全部重置则清空所有捕获的时间。

    状态管理：面板按钮会根据操作流程（如加载前、加载后、保存后）智能切换可用状态，引导用户正确操作。

    多配置切换：支持在不同任务间随时加载新的 JSON 配置文件，时间轴和参数会即时更新。

4. 步骤四：JSON 文件详解
4.1 标准结构

JSON

{
  "case_time": {
    "start_time": 1234567890.123456,
    "takeover": 1234567895.234567,
    "event": 1234567900.345678,
    "end_time": 1234567905.456789
  },
  "vehicle_config": "JLBJA47708D",
  "case_info": {
    "ego_initial": { "position": {...}, "rotation": {...} },
    "ego_dest": {...},
    "scene": "environment_GuoZhan"
  },
  "db.sqlite": "https://..."
}

4.2 字段说明

此插件主要关注并编辑以下字段：

字段路径
	

颜色标识
	

类型
	

说明

case_time.start_time
	

浅绿色
	

Number
	

场景的起始时间戳

case_time.takeover
	

浅黄色
	

Number
	

人工接管或关键控制事件的时间戳

case_time.event
	

浅橙色
	

Number
	

需要特别关注的关键事件时间戳

case_time.end_time
	

浅红色
	

Number
	

场景的结束时间戳

vehicle_config
	

浅灰色
	

String
	

车辆的唯一标识或配置信息

注意：插件会完整保留并回写 case_info 等其他字段，仅对上述字段进行修改。
5. 使用技巧与最佳实践

    工作流建议：加载 → 准备Rosbag → 标注 → 保存。

    精度：本工具支持微秒级时间精度（小数点后6位），请在手动输入时注意格式。

    文件管理：为不同场景或日期的标注任务创建独立的 JSON 文件，并使用有意义的命名，便于追溯和管理。

    多任务处理：利用插件保存后不清空、支持随时加载新文件的特性，可以流畅地在多个标注任务之间切换。

6. 技术支持

    系统要求:

        OS: Ubuntu 20.04

        ROS: ROS Noetic

    获取帮助:

        优先查阅项目中的 README.md 和其他文档。

        如遇问题，请参考本文档的 故障排除指南。

文档最后更新：2025年8月12日
