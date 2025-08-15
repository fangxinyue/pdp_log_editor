#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_list_macros.h>
#include "pdp_log_editor/pdp_log_editor_panel.h"
#include <fstream>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>
#include <nlohmann/json.hpp>
#include "pdp_log_editor/PdpLogAnnotation.h"
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QLineEdit> // 新增
#include <QPainter>
#include <QMouseEvent>
#include <QTimer>
#include <ros/ros.h>
#include <limits>
#include <algorithm>


namespace pdp_log_editor {

PdpLogEditorPanel::PdpLogEditorPanel(QWidget* parent) : rviz::Panel(parent), clicks_done_(0), manual_clicks_done_(0), use_bag_time_(false),
    timeline_start_time_(0.0), timeline_end_time_(100.0), current_timeline_time_(0.0), timeline_dragging_(false),
    user_operation_delay_(0) {
    auto* layout = new QVBoxLayout(this);

    status_label_ = new QLabel("请在3D视图中激活工具并开始标注...");
    status_label_->setStyleSheet("font-weight: bold; color: blue;");
    layout->addWidget(status_label_);

    auto* grid_layout = new QGridLayout();
    
    // start_time - Green
    grid_layout->addWidget(new QLabel("start_time"), 0, 0);
    start_time_spinbox_ = new QDoubleSpinBox();
    start_time_spinbox_->setRange(-999999999999.99, 999999999999.99);
    start_time_spinbox_->setDecimals(6);  // 增加精度以支持完整的bag时间
    start_time_spinbox_->setValue(0.0);
    start_time_spinbox_->setStyleSheet("background-color: #90EE90; font-weight: bold;"); // 浅绿色
    grid_layout->addWidget(start_time_spinbox_, 0, 1);

    // takeover - Yellow
    grid_layout->addWidget(new QLabel("takeover"), 1, 0);
    takeover_time_spinbox_ = new QDoubleSpinBox();
    takeover_time_spinbox_->setRange(-999999999999.99, 999999999999.99);
    takeover_time_spinbox_->setDecimals(6);  // 增加精度以支持完整的bag时间
    takeover_time_spinbox_->setValue(0.0);
    takeover_time_spinbox_->setStyleSheet("background-color: #FFFF99; font-weight: bold;"); // 浅黄色
    grid_layout->addWidget(takeover_time_spinbox_, 1, 1);

    // event - Orange
    grid_layout->addWidget(new QLabel("event"), 2, 0);
    event_time_spinbox_ = new QDoubleSpinBox();
    event_time_spinbox_->setRange(-999999999999.99, 999999999999.99);
    event_time_spinbox_->setDecimals(6);  // 增加精度以支持完整的bag时间
    event_time_spinbox_->setValue(0.0);
    event_time_spinbox_->setStyleSheet("background-color: #FFB366; font-weight: bold;"); // 浅橙色
    grid_layout->addWidget(event_time_spinbox_, 2, 1);

    // end_time - Red
    grid_layout->addWidget(new QLabel("end_time"), 3, 0);
    end_time_spinbox_ = new QDoubleSpinBox();
    end_time_spinbox_->setRange(-999999999999.99, 999999999999.99);
    end_time_spinbox_->setDecimals(6);  // 增加精度以支持完整的bag时间
    end_time_spinbox_->setValue(0.0);
    end_time_spinbox_->setStyleSheet("background-color: #FFB3B3; font-weight: bold;"); // 浅红色
    grid_layout->addWidget(end_time_spinbox_, 3, 1);

    // vehicle_config - Gray
    grid_layout->addWidget(new QLabel("vehicle_config"), 4, 0);
    vehicle_config_edit_ = new QLineEdit();
    vehicle_config_edit_->setText("JLBJA47708D"); // 默认值
    vehicle_config_edit_->setStyleSheet("background-color: #E0E0E0;"); // 浅灰色
    grid_layout->addWidget(vehicle_config_edit_, 4, 1);

    layout->addLayout(grid_layout);

    undo_button_ = new QPushButton("撤销上一步");
    reset_button_ = new QPushButton("全部重置");
    layout->addWidget(undo_button_);
    layout->addWidget(reset_button_);

    save_button_ = new QPushButton("Save to JSON");
    load_button_ = new QPushButton("Load from JSON");
    manual_capture_button_ = new QPushButton("手动获取时间 (0/2)");
    manual_capture_button_->setStyleSheet("font-weight: bold; color: green;");
    
    layout->addWidget(save_button_);
    layout->addWidget(load_button_);
    layout->addWidget(manual_capture_button_);

    // 新增：用于显示JSON路径的标签
    json_path_label_ = new QLabel("case json path: ");
    layout->addWidget(json_path_label_);

    // 添加时间轴部分
    auto* timeline_group = new QVBoxLayout();
    
    // 当前时间显示
    current_time_label_ = new QLabel("当前时间: 0.00s");
    current_time_label_->setStyleSheet("font-weight: bold; color: blue;");
    timeline_group->addWidget(current_time_label_);
    
    // 时间轴可视化Widget
    timeline_widget_ = new TimelineWidget(this);
    timeline_widget_->setMinimumHeight(80);
    timeline_widget_->setMaximumHeight(120);
    timeline_group->addWidget(timeline_widget_);
    
    // 时间轴滑动条（进度条）
    timeline_slider_ = new QSlider(Qt::Horizontal);
    timeline_slider_->setMinimum(0);
    timeline_slider_->setMaximum(10000);
    timeline_slider_->setValue(0);
    timeline_slider_->setStyleSheet("QSlider::groove:horizontal { height: 8px; background: #ddd; border-radius: 4px; } QSlider::handle:horizontal { background: #007acc; border: 1px solid #005a9e; width: 18px; margin: -2px 0; border-radius: 3px; }");
    timeline_group->addWidget(timeline_slider_);
    
    layout->addLayout(timeline_group);

    connect(undo_button_, &QPushButton::clicked, this, &PdpLogEditorPanel::onUndo);
    connect(reset_button_, &QPushButton::clicked, this, &PdpLogEditorPanel::onReset);
    connect(save_button_, &QPushButton::clicked, this, &PdpLogEditorPanel::onSaveToJson);
    connect(load_button_, &QPushButton::clicked, this, &PdpLogEditorPanel::onLoadFromJson);
    connect(manual_capture_button_, &QPushButton::clicked, this, &PdpLogEditorPanel::onManualTimeCapture);
    connect(timeline_slider_, &QSlider::valueChanged, this, &PdpLogEditorPanel::onTimelineChanged);
    connect(timeline_widget_, &TimelineWidget::timestampClicked, this, &PdpLogEditorPanel::onTimestampClicked);
    
    // 连接时间编辑框的信号
    connect(start_time_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &PdpLogEditorPanel::onStartTimeChanged);
    connect(takeover_time_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &PdpLogEditorPanel::onTakeoverTimeChanged);
    connect(event_time_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &PdpLogEditorPanel::onEventTimeChanged);
    connect(end_time_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &PdpLogEditorPanel::onEndTimeChanged);
    
    setLayout(layout);
    
    // 初始化定时器
    timeline_update_timer_ = new QTimer(this);
    connect(timeline_update_timer_, &QTimer::timeout, this, &PdpLogEditorPanel::updateTimelineDisplay);
    timeline_update_timer_->start(100); // 优化：降低到10Hz (100ms) 更新，显著降低CPU负载
    
    updateUI();
    initializeTimeline();
    
    // 设置初始按钮状态：只有Load from JSON可用
    setInitialButtonStates();
    
    // 新增：创建用于轮询/sim_pause参数的定时器
    param_check_timer_ = nh_.createTimer(ros::Duration(0.2), &PdpLogEditorPanel::checkSimPauseParam, this);
}

PdpLogEditorPanel::~PdpLogEditorPanel() {}

void PdpLogEditorPanel::onInitialize() {
    rviz::Panel::onInitialize();
    ros::NodeHandle nh;
    click_update_sub_ = nh.subscribe("/pdp_log_editor/internal/click_update", 1, &PdpLogEditorPanel::clickUpdateCallback, this);
    control_cmd_pub_ = nh.advertise<std_msgs::String>("/pdp_log_editor/internal/control_cmd", 1);
    annotation_result_pub_ = nh.advertise<pdp_log_editor::PdpLogAnnotation>("/pdp_log_editor/annotation_result", 1, true);
    click_event_pub_ = nh.advertise<pdp_log_editor::PdpLogAnnotation>("/pdp_log_editor/internal/click_update", 1);  // 新增：用于手动时间发布
    
    // 订阅/clock话题用于同步时间轴
    // 优化：减小队列大小为1，因为我们只关心最新的时间，旧的可以直接丢弃
    clock_sub_ = nh.subscribe("/clock", 1, &PdpLogEditorPanel::clockCallback, this);
}

void PdpLogEditorPanel::onUndo() {
    // 优先处理手动点击的撤销
    if (manual_clicks_done_ > 0) {
        manual_clicks_done_--;
        
        // 清除对应的时间戳
        switch (manual_clicks_done_) {
            case 0:
                current_annotation_.takeover_time = ros::Time(0);
                status_label_->setText("Undo takeover");
                break;
            case 1:
                current_annotation_.event_time = ros::Time(0);
                status_label_->setText("已撤销事件时间");
                break;
        }
        
        // 更新按钮状态
        manual_capture_button_->setText(QString("手动获取时间 (%1/2)").arg(manual_clicks_done_));
        manual_capture_button_->setStyleSheet("font-weight: bold; color: green;");
        
        // 同步点击计数
        clicks_done_ = manual_clicks_done_;
        
        updateUI();
        
        // 发布更新
        click_event_pub_.publish(current_annotation_);
    } else {
        // 如果没有手动点击，则发送撤销命令給工具
        std_msgs::String cmd;
        cmd.data = "UNDO";
        control_cmd_pub_.publish(cmd);
    }
}

void PdpLogEditorPanel::onReset() {
    std_msgs::String cmd;
    cmd.data = "RESET";
    control_cmd_pub_.publish(cmd);
    
    // 重置手动捕获状态
    resetManualCapture();
    status_label_->setText("已重置所有标注数据");
}

void PdpLogEditorPanel::onManualTimeCapture() {
    if (manual_clicks_done_ >= 2) {
        status_label_->setText("已完成2次时间捕获，请先重置！");
        return;
    }
    
    ros::Time current_time = getCurrentTime();
    double time_value = current_time.toSec();
    
    // 检测时间来源
    QString time_source = "ROS系统时间";
    try {
        rosgraph_msgs::ClockConstPtr clock_msg = ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", ros::Duration(0.1));
        if (clock_msg && abs(clock_msg->clock.toSec() - time_value) < 0.1) {
            time_source = "Rosbag仿真时间";
        }
    } catch (...) {
        // 使用默认的ROS系统时间标识
    }
    
    // 根据点击次数设置对应的时间并更新相应的编辑框
    switch (manual_clicks_done_) {
        case 0:
            current_annotation_.takeover_time = current_time;
            takeover_time_spinbox_->blockSignals(true);
            takeover_time_spinbox_->setValue(time_value);
            takeover_time_spinbox_->blockSignals(false);
            status_label_->setText(QString("Captured takeover: %1s (%2)").arg(time_value, 0, 'f', 3).arg(time_source));
            break;
        case 1:
            current_annotation_.event_time = current_time;
            event_time_spinbox_->blockSignals(true);
            event_time_spinbox_->setValue(time_value);
            event_time_spinbox_->blockSignals(false);
            status_label_->setText(QString("已捕获事件时间: %1s (%2)").arg(time_value, 0, 'f', 3).arg(time_source));
            break;
    }
    
    manual_clicks_done_++;
    clicks_done_ = manual_clicks_done_;  // 同步点击计数
    
    // 更新按钮文本
    manual_capture_button_->setText(QString("手动获取时间 (%1/2)").arg(manual_clicks_done_));
    
    if (manual_clicks_done_ >= 2) {
        manual_capture_button_->setText("手动获取完成 (2/2)");
        manual_capture_button_->setStyleSheet("font-weight: bold; color: blue;");
        status_label_->setText("手动时间捕获完成，可点击发布！");
    }
    
    updateUI();
    
    // 发布更新给其他组件
    click_event_pub_.publish(current_annotation_);
}

void PdpLogEditorPanel::clickUpdateCallback(const pdp_log_editor::PdpLogAnnotation::ConstPtr& msg) {
    current_annotation_ = *msg;
    clicks_done_ = 0;
    if (msg->takeover_time.toSec() > 0) clicks_done_++;
    if (msg->event_time.toSec() > 0) clicks_done_++;
    updateUI();
}

void PdpLogEditorPanel::updateUI() {
    // 更新可编辑时间框的显示（阻塞信号防止循环触发）
    start_time_spinbox_->blockSignals(true);
    start_time_spinbox_->setValue(current_annotation_.scene_start_time.toSec());
    start_time_spinbox_->blockSignals(false);
    
    takeover_time_spinbox_->blockSignals(true);
    takeover_time_spinbox_->setValue(current_annotation_.takeover_time.toSec());
    takeover_time_spinbox_->blockSignals(false);
    
    event_time_spinbox_->blockSignals(true);
    event_time_spinbox_->setValue(current_annotation_.event_time.toSec());
    event_time_spinbox_->blockSignals(false);
    
    end_time_spinbox_->blockSignals(true);
    end_time_spinbox_->setValue(current_annotation_.scene_end_time.toSec());
    end_time_spinbox_->blockSignals(false);

    switch (clicks_done_) {
        case 0: status_label_->setText("请点击手动获取【控车时间】"); break;
        case 1: status_label_->setText("请点击手动获取【事件时间】"); break;
        case 2: status_label_->setText("手动时间捕获完成，可编辑或保存"); break;
        default: break;
    }
    
    // 更新时间轴显示
    updateTimelinePosition();
}

void PdpLogEditorPanel::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

void PdpLogEditorPanel::load(const rviz::Config& config) {
    rviz::Panel::load(config);
}

void PdpLogEditorPanel::onSaveToJson() {
    // if (clicks_done_ < 4) {
    //     status_label_->setText("标注未完成，无法保存！");
    //     return;
    // }

    QString file_path = QFileDialog::getSaveFileName(this, 
        "保存标注为JSON文件", 
        QDir::homePath(), 
        "JSON文件 (*.json)");
    
    if (file_path.isEmpty()) {
        return;
    }

    try {
        nlohmann::json j;
        
        // 检查是否已存在JSON文件，如果存在则加载现有内容
        std::ifstream existing_file(file_path.toStdString());
        if (existing_file.good()) {
            try {
                existing_file >> j;
                existing_file.close();
            } catch (...) {
                // 如果读取失败，使用默认结构
                j.clear();
            }
        }
        
        // 只更新时间字段，保持其他字段不变，按照正确的顺序
        j["case_time"]["start_time"] = current_annotation_.scene_start_time.toSec();
        j["case_time"]["takeover"] = current_annotation_.takeover_time.toSec();
        j["case_time"]["event"] = current_annotation_.event_time.toSec();
        j["case_time"]["end_time"] = current_annotation_.scene_end_time.toSec();
        
        // 更新 vehicle_config
        j["vehicle_config"] = vehicle_config_edit_->text().toStdString();

        // 如果是新文件，添加默认的其他字段
        if (!j.contains("case_info")) {
            j["case_info"]["ego_initial"]["position"]["x"] = -4164.322722157463;
            j["case_info"]["ego_initial"]["position"]["y"] = 0.0;
            j["case_info"]["ego_initial"]["position"]["z"] = 488.3309675815399;
            j["case_info"]["ego_initial"]["rotation"]["x"] = 0.0;
            j["case_info"]["ego_initial"]["rotation"]["y"] = 250.71370161918355;
            j["case_info"]["ego_initial"]["rotation"]["z"] = 0.0;
            j["case_info"]["ego_dest"]["x"] = -4229.899380000308;
            j["case_info"]["ego_dest"]["y"] = 0.0;
            j["case_info"]["ego_dest"]["z"] = 183.6990550000337;
            j["case_info"]["scene"] = "environment_GuoZhan";
        }
        
        if (!j.contains("db.sqlite")) {
            j["db.sqlite"] = "https://autocar-mogosim-1255510688.cos.ap-beijing.myqcloud.com/hadmap/bj-V2.9.8.sqlite";
        }

        std::ofstream out(file_path.toStdString());
        out << j.dump(4);
        out.close();

        status_label_->setText("标注已保存到JSON文件！");
        json_path_label_->setText("case json path: " + file_path);
        
        // 更新UI状态，确保按钮等界面元素状态正确
        updateUI();
        
    } catch (const std::exception& e) {
        status_label_->setText(QString("保存失败: ") + e.what());
    }
}

void PdpLogEditorPanel::onLoadFromJson() {
    QString file_path = QFileDialog::getOpenFileName(this,
        "从JSON文件加载标注",
        QDir::homePath(),
        "JSON文件 (*.json)");

    if (file_path.isEmpty()) {
        return;
    }

    try {
        std::ifstream in(file_path.toStdString());
        nlohmann::json j;
        in >> j;

        // 支持新格式和旧格式
        if (j.contains("case_time")) {
            // 新格式 - 处理可能为空字符串的时间字段
            auto loadTime = [](const nlohmann::json& timeField) -> double {
                if (timeField.is_string() && timeField.get<std::string>().empty()) {
                    return 0.0;  // 空字符串转为0
                } else if (timeField.is_number()) {
                    return timeField.get<double>();
                } else {
                    return 0.0;  // 其他情况默认为0
                }
            };
            
            current_annotation_.scene_start_time.fromSec(loadTime(j["case_time"]["start_time"]));
            current_annotation_.takeover_time.fromSec(loadTime(j["case_time"]["takeover"]));
            current_annotation_.event_time.fromSec(loadTime(j["case_time"]["event"]));
            current_annotation_.scene_end_time.fromSec(loadTime(j["case_time"]["end_time"]));
        } else {
            // 兼容旧格式
            current_annotation_.scene_start_time.fromSec(j["scene_start_time"]);
            current_annotation_.takeover_time.fromSec(j["takeover_time"]);
            current_annotation_.event_time.fromSec(j["event_time"]);
            current_annotation_.scene_end_time.fromSec(j["scene_end_time"]);
        }

        // 新增：加载 vehicle_config
        if (j.contains("vehicle_config") && j["vehicle_config"].is_string()) {
            vehicle_config_edit_->setText(QString::fromStdString(j["vehicle_config"]));
        }

        // 计算已完成的点击次数（只计算takeover和event）
        clicks_done_ = 0;
        manual_clicks_done_ = 0;
        if (current_annotation_.takeover_time.toSec() > 0) { clicks_done_++; manual_clicks_done_++; }
        if (current_annotation_.event_time.toSec() > 0) { clicks_done_++; manual_clicks_done_++; }
        
        // 更新手动捕获按钮状态
        if (manual_clicks_done_ >= 2) {
            manual_capture_button_->setText("手动获取完成 (2/2)");
            manual_capture_button_->setStyleSheet("font-weight: bold; color: blue;");
        } else {
            manual_capture_button_->setText(QString("手动获取时间 (%1/2)").arg(manual_clicks_done_));
            manual_capture_button_->setStyleSheet("font-weight: bold; color: green;");
        }
        
        updateUI();
        
        // 启用所有按钮和编辑框
        enableAllButtons();
        
        // 自动设置时间轴范围基于加载的时间戳
        double min_time = std::numeric_limits<double>::max();
        double max_time = std::numeric_limits<double>::lowest();
        
        // 收集所有非零时间戳
        std::vector<double> times;
        if (current_annotation_.scene_start_time.toSec() > 0) times.push_back(current_annotation_.scene_start_time.toSec());
        if (current_annotation_.takeover_time.toSec() > 0) times.push_back(current_annotation_.takeover_time.toSec());
        if (current_annotation_.event_time.toSec() > 0) times.push_back(current_annotation_.event_time.toSec());
        if (current_annotation_.scene_end_time.toSec() > 0) times.push_back(current_annotation_.scene_end_time.toSec());
        
        if (!times.empty()) {
            min_time = *std::min_element(times.begin(), times.end());
            max_time = *std::max_element(times.begin(), times.end());
            
            // 添加一些缓冲区，让时间轴更方便使用
            double buffer = (max_time - min_time) * 0.1; // 10%的缓冲区
            if (buffer < 10.0) buffer = 10.0; // 最少10秒缓冲区
            
            timeline_start_time_ = min_time - buffer;
            timeline_end_time_ = max_time + buffer;
            current_timeline_time_ = min_time; // 设置当前时间为开始时间
            
            // 更新时间轴显示
            timeline_widget_->setTimeRange(timeline_start_time_, timeline_end_time_);
            updateTimelinePosition();
            updateTimelineDisplay();
            
            status_label_->setText(QString("标注已从JSON文件加载！(%1/2 完成) 时间轴: %2s - %3s")
                .arg(clicks_done_)
                .arg(timeline_start_time_, 0, 'f', 2)
                .arg(timeline_end_time_, 0, 'f', 2));
        } else {
            status_label_->setText(QString("标注已从JSON文件加载！(%1/2 完成)").arg(clicks_done_));
        }
        
        json_path_label_->setText("case json path: " + file_path);
    } catch (const std::exception& e) {
        status_label_->setText(QString("加载失败: ") + e.what());
    }
}

void PdpLogEditorPanel::resetManualCapture() {
    manual_clicks_done_ = 0;
    manual_capture_button_->setText("手动获取时间 (0/2)");
    manual_capture_button_->setStyleSheet("font-weight: bold; color: green;");
    
    // 清除手动捕获的时间（保留start_time和end_time）
    current_annotation_.takeover_time = ros::Time(0);
    current_annotation_.event_time = ros::Time(0);
    
    clicks_done_ = 0;
    updateUI();
    
    // 发布清空的状态
    click_event_pub_.publish(current_annotation_);
}

ros::Time PdpLogEditorPanel::getCurrentTime() {
    // 优先从/clock话题获取仿真时间（rosbag播放时间）
    try {
        rosgraph_msgs::ClockConstPtr clock_msg = ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", ros::Duration(0.1));
        if (clock_msg) {
            return clock_msg->clock;
        }
    } catch (...) {
        // 如果无法获取/clock话题，说明没有rosbag在播放
    }
    
    // 如果没有/clock话题，返回ROS系统时间
    return ros::Time::now();
}

void PdpLogEditorPanel::onTimelineChanged(int value) {
    current_timeline_time_ = sliderValueToTimestamp(value);
    current_time_label_->setText(QString("当前时间: %1s").arg(current_timeline_time_, 0, 'f', 2));
    timeline_widget_->setCurrentTime(current_timeline_time_);
    
    // 检查rosbag进程是否存在
    int rosbag_running = system("pgrep -f 'rosbag play' > /dev/null 2>&1");
    
    if (rosbag_running != 0) {
        status_label_->setText("警告：未检测到rosbag play进程，无法跳转时间");
        return;
    }
    
    // 直接的rosbag交互式命令
    std::string interactive_cmd = "s " + std::to_string(current_timeline_time_);
    int result = system(("echo '" + interactive_cmd + "\\n' > /proc/$(pgrep -f 'rosbag play')/fd/0 2>/dev/null").c_str());
    
    if (result == 0) {
        status_label_->setText(QString("跳转到时间: %1s").arg(current_timeline_time_, 0, 'f', 2));
        ROS_INFO("成功发送时间跳转命令: %f", current_timeline_time_);
    } else {
        status_label_->setText(QString("时间跳转可能失败，目标时间: %1s").arg(current_timeline_time_, 0, 'f', 2));
        ROS_WARN("Failed to send seek command to rosbag process");
    }
}

void PdpLogEditorPanel::onTimestampClicked(int timestamp_type) {
    // 将当前时间轴时间设置为指定的时间戳
    switch (timestamp_type) {
        case 0: // start_time
            current_annotation_.scene_start_time.fromSec(current_timeline_time_);
            status_label_->setText(QString("Set start_time: %1s").arg(current_timeline_time_, 0, 'f', 2));
            break;
        case 1: // takeover
            current_annotation_.takeover_time.fromSec(current_timeline_time_);
            status_label_->setText(QString("已设置控车时间: %1s").arg(current_timeline_time_, 0, 'f', 2));
            break;
        case 2: // event
            current_annotation_.event_time.fromSec(current_timeline_time_);
            status_label_->setText(QString("已设置事件时间: %1s").arg(current_timeline_time_, 0, 'f', 2));
            break;
        case 3: // end_time
            current_annotation_.scene_end_time.fromSec(current_timeline_time_);
            status_label_->setText(QString("已设置场景结束时间: %1s").arg(current_timeline_time_, 0, 'f', 2));
            break;
    }
    
    updateUI();
    updateTimelinePosition();
    
    // 发布更新
    click_event_pub_.publish(current_annotation_);
}

void PdpLogEditorPanel::initializeTimeline() {
    // 初始化时间轴范围（可以从ROS参数或配置文件读取）
    timeline_start_time_ = 0.0;
    timeline_end_time_ = 300.0; // 默认5分钟
    
    timeline_widget_->setTimeRange(timeline_start_time_, timeline_end_time_);
    updateTimelinePosition();
}

void PdpLogEditorPanel::updateTimelinePosition() {
    // 更新时间轴上的时间戳显示
    timeline_widget_->setTimestamps(
        current_annotation_.scene_start_time.toSec(),
        current_annotation_.takeover_time.toSec(),
        current_annotation_.event_time.toSec(),
        current_annotation_.scene_end_time.toSec()
    );
    
    // 更新滑动条位置
    int slider_pos = timestampToSliderValue(current_timeline_time_);
    timeline_slider_->blockSignals(true);
    timeline_slider_->setValue(slider_pos);
    timeline_slider_->blockSignals(false);
}

double PdpLogEditorPanel::timestampToSliderValue(double timestamp) {
    if (timeline_end_time_ <= timeline_start_time_) return 0;
    
    double ratio = (timestamp - timeline_start_time_) / (timeline_end_time_ - timeline_start_time_);
    ratio = std::max(0.0, std::min(1.0, ratio));
    return ratio * timeline_slider_->maximum();
}

double PdpLogEditorPanel::sliderValueToTimestamp(int slider_value) {
    double ratio = double(slider_value) / timeline_slider_->maximum();
    return timeline_start_time_ + ratio * (timeline_end_time_ - timeline_start_time_);
}

//////////////////////////////////////////
// TimelineWidget 实现
//////////////////////////////////////////

TimelineWidget::TimelineWidget(QWidget* parent) : QWidget(parent), start_time_(0), end_time_(100), current_time_(0), 
    widget_width_(400), dragging_timestamp_(-1) {
    
    // 设置不同时间戳的颜色
    timestamp_colors_[0] = QColor(0, 255, 0);     // 场景开始 - 绿色
    timestamp_colors_[1] = QColor(255, 255, 0);   // 控车时间 - 黄色  
    timestamp_colors_[2] = QColor(255, 165, 0);   // 事件时间 - 橙色
    timestamp_colors_[3] = QColor(255, 0, 0);     // 场景结束 - 红色
    
    // 初始化时间戳
    for (int i = 0; i < 4; i++) {
        timestamps_[i] = 0.0;
    }
    
    setMouseTracking(true);
}

void TimelineWidget::setTimeRange(double start_time, double end_time) {
    start_time_ = start_time;
    end_time_ = end_time;
    update();
}

void TimelineWidget::setTimestamps(double start, double takeover, double event, double end) {
    timestamps_[0] = start;
    timestamps_[1] = takeover;
    timestamps_[2] = event;
    timestamps_[3] = end;
    update();
}

void TimelineWidget::setCurrentTime(double current_time) {
    current_time_ = current_time;
    update();
}

void TimelineWidget::paintEvent(QPaintEvent* event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    widget_width_ = width();
    int height = this->height();
    int timeline_y = height / 2;
    int margin = 20;
    
    // 绘制时间轴背景
    painter.setPen(QPen(Qt::black, 2));
    painter.drawLine(margin, timeline_y, widget_width_ - margin, timeline_y);
    
    // 绘制时间刻度
    painter.setPen(QPen(Qt::gray, 1));
    QFont font = painter.font();
    font.setPointSize(8);
    painter.setFont(font);
    
    for (int i = 0; i <= 10; i++) {
        int x = margin + i * (widget_width_ - 2 * margin) / 10;
        painter.drawLine(x, timeline_y - 5, x, timeline_y + 5);
        
        double time_value = start_time_ + i * (end_time_ - start_time_) / 10;
        painter.drawText(x - 15, timeline_y + 20, QString::number(time_value, 'f', 1));
    }
    
    // 绘制当前时间指示器
    if (current_time_ >= start_time_ && current_time_ <= end_time_) {
        int current_x = timeToPixel(current_time_);
        painter.setPen(QPen(Qt::blue, 3));
        painter.drawLine(current_x, 10, current_x, height - 10);
        
        // 绘制当前时间标签
        painter.setPen(QPen(Qt::blue));
        painter.drawText(current_x - 20, 8, QString("%1s").arg(current_time_, 0, 'f', 1));
    }
    
    // 绘制时间戳标记
    for (int i = 0; i < 4; i++) {
        if (timestamps_[i] > 0 && timestamps_[i] >= start_time_ && timestamps_[i] <= end_time_) {
            int x = timeToPixel(timestamps_[i]);
            
            // 绘制时间戳标记
            painter.setPen(QPen(timestamp_colors_[i], 2));
            painter.setBrush(QBrush(timestamp_colors_[i]));
            
            // 绘制菱形标记
            QPolygon diamond;
            diamond << QPoint(x, timeline_y - 10) << QPoint(x + 8, timeline_y) 
                   << QPoint(x, timeline_y + 10) << QPoint(x - 8, timeline_y);
            painter.drawPolygon(diamond);
            
            // 绘制时间戳标签
            painter.setPen(QPen(timestamp_colors_[i]));
            QString label;
            switch (i) {
                case 0: label = "开始"; break;
                case 1: label = "控车"; break;
                case 2: label = "事件"; break;
                case 3: label = "结束"; break;
            }
            painter.drawText(x - 15, timeline_y - 15, label);
            painter.drawText(x - 20, timeline_y + 30, QString::number(timestamps_[i], 'f', 1));
        }
    }
}

void TimelineWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        int clicked_timestamp = getTimestampAtPosition(event->x(), event->y());
        if (clicked_timestamp >= 0) {
            dragging_timestamp_ = clicked_timestamp;
        } else {
            // 点击时间轴设置当前时间
            double clicked_time = pixelToTime(event->x());
            emit timestampClicked(-1); // 表示点击时间轴，而不是特定时间戳
        }
    }
}

void TimelineWidget::mouseMoveEvent(QMouseEvent* event) {
    if (dragging_timestamp_ >= 0) {
        double new_time = pixelToTime(event->x());
        new_time = std::max(start_time_, std::min(end_time_, new_time));
        timestamps_[dragging_timestamp_] = new_time;
        update();
        
        // 发送时间戳更改信号
        emit timestampClicked(dragging_timestamp_);
    }
}

void TimelineWidget::resizeEvent(QResizeEvent* event) {
    widget_width_ = width();
    update();
}

double TimelineWidget::pixelToTime(int pixel_x) {
    int margin = 20;
    int effective_width = widget_width_ - 2 * margin;
    if (effective_width <= 0) return start_time_;
    
    double ratio = double(pixel_x - margin) / effective_width;
    ratio = std::max(0.0, std::min(1.0, ratio));
    return start_time_ + ratio * (end_time_ - start_time_);
}

int TimelineWidget::timeToPixel(double time) {
    int margin = 20;
    int effective_width = widget_width_ - 2 * margin;
    if (end_time_ <= start_time_) return margin;
    
    double ratio = (time - start_time_) / (end_time_ - start_time_);
    return margin + int(ratio * effective_width);
}

int TimelineWidget::getTimestampAtPosition(int x, int y) {
    int timeline_y = height() / 2;
    
    for (int i = 0; i < 4; i++) {
        if (timestamps_[i] > 0) {
            int timestamp_x = timeToPixel(timestamps_[i]);
            if (abs(x - timestamp_x) < 15 && abs(y - timeline_y) < 15) {
                return i;
            }
        }
    }
    return -1;
}

// 可编辑时间框的槽函数实现
void PdpLogEditorPanel::onStartTimeChanged(double value) {
    current_annotation_.scene_start_time.fromSec(value);
    updateUI();
    // 发布更新
    click_event_pub_.publish(current_annotation_);
}

void PdpLogEditorPanel::onTakeoverTimeChanged(double value) {
    current_annotation_.takeover_time.fromSec(value);
    updateUI();
    // 发布更新
    click_event_pub_.publish(current_annotation_);
}

void PdpLogEditorPanel::onEventTimeChanged(double value) {
    current_annotation_.event_time.fromSec(value);
    updateUI();
    // 发布更新
    click_event_pub_.publish(current_annotation_);
}

void PdpLogEditorPanel::onEndTimeChanged(double value) {
    current_annotation_.scene_end_time.fromSec(value);
    updateUI();
    // 发布更新
    click_event_pub_.publish(current_annotation_);
}

// /clock话题回调 - 用于与rosbag同步
void PdpLogEditorPanel::clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg) {
    // 优化：此回调函数现在只更新内部时间变量，不做任何UI操作。
    // 所有的UI更新都由低频率的QTimer (updateTimelineDisplay) 统一处理。
    // 这将高频的ROS消息处理与低频的UI渲染解耦，是解决卡顿的关键。
    current_timeline_time_ = msg->clock.toSec();
}

// 更新时间轴显示
void PdpLogEditorPanel::updateTimelineDisplay() {
    // 如果时间超出范围，动态调整时间轴范围
    if (current_timeline_time_ > timeline_end_time_) {
        timeline_end_time_ = current_timeline_time_ + 60.0; // 延长60秒缓冲区
        timeline_widget_->setTimeRange(timeline_start_time_, timeline_end_time_);
    }
    if (current_timeline_time_ < timeline_start_time_ && current_timeline_time_ > 0) { // 避免重置为0
        timeline_start_time_ = current_timeline_time_ - 10.0; // 提前10秒
        if (timeline_start_time_ < 0) timeline_start_time_ = 0;
        timeline_widget_->setTimeRange(timeline_start_time_, timeline_end_time_);
    }

    // 更新当前时间显示
    current_time_label_->setText(QString("当前时间: %1s").arg(current_timeline_time_, 0, 'f', 3)); // 增加一位小数
    
    // 更新时间轴widget的当前时间
    timeline_widget_->setCurrentTime(current_timeline_time_);
    
    // 更新进度条位置（阻塞信号防止循环）
    if (!timeline_slider_->isSliderDown()) { // 只有在用户没有拖动时才更新
        timeline_slider_->blockSignals(true);
        int slider_pos = timestampToSliderValue(current_timeline_time_);
        timeline_slider_->setValue(slider_pos);
        timeline_slider_->blockSignals(false);
    }
}

void PdpLogEditorPanel::onLoadBagInfo() {
    QString file_path = QFileDialog::getOpenFileName(this,
        "选择Rosbag文件",
        QDir::homePath(),
        "Rosbag文件 (*.bag)");

    if (file_path.isEmpty()) {
        return;
    }

    try {
        rosbag::Bag bag;
        bag.open(file_path.toStdString(), rosbag::bagmode::Read);

        rosbag::View view(bag);
        ros::Time start_time = view.getBeginTime();
        ros::Time end_time = view.getEndTime();
        
        bag.close();

        if (start_time.isZero() || end_time.isZero() || start_time > end_time) {
             QMessageBox::warning(this, "错误", "无法读取有效的rosbag时间范围。");
             return;
        }

        timeline_start_time_ = start_time.toSec();
        timeline_end_time_ = end_time.toSec();
        current_timeline_time_ = timeline_start_time_;

        timeline_widget_->setTimeRange(timeline_start_time_, timeline_end_time_);
        updateTimelinePosition();
        updateTimelineDisplay();

        status_label_->setText(QString("已加载Rosbag时间轴: %1s - %2s")
            .arg(timeline_start_time_, 0, 'f', 2)
            .arg(timeline_end_time_, 0, 'f', 2));

    } catch (const rosbag::BagException& e) {
        QMessageBox::critical(this, "Rosbag读取错误", QString("无法打开或解析rosbag文件: %1").arg(e.what()));
    }
}

void PdpLogEditorPanel::checkSimPauseParam(const ros::TimerEvent&) {
    bool is_paused = false;
    if (nh_.getParam("/sim_pause", is_paused)) {
        toggleRosbagPlayback(is_paused);
    }
}

void PdpLogEditorPanel::toggleRosbagPlayback(bool pause) {
    static bool last_pause_state = false;
    if (pause == last_pause_state) {
        return; // 状态未改变，无需操作
    }

    const char* find_pid_cmd = "ps aux | grep 'rosbag play' | grep -v grep | awk '{print $2}' | head -n 1";
    
    FILE* pipe = popen(find_pid_cmd, "r");
    if (!pipe) {
        ROS_ERROR("Failed to run command to find rosbag PID.");
        return;
    }

    char buffer[128];
    std::string pid_str = "";
    if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        pid_str = buffer;
        pid_str.erase(pid_str.find_last_not_of(" \n\r\t")+1);
    }
    pclose(pipe);

    if (pid_str.empty()) {
        // ROS_WARN("No running 'rosbag play' process found.");
        return;
    }

    std::string toggle_cmd = "echo ' ' > /proc/" + pid_str + "/fd/0";
    
    int result = system(toggle_cmd.c_str());

    if (result == 0) {
        if (pause) {
            status_label_->setText("Rosbag已通过参数暂停");
            ROS_INFO("Rosbag playback paused via /sim_pause parameter.");
        } else {
            status_label_->setText("Rosbag已通过参数恢复");
            ROS_INFO("Rosbag playback resumed via /sim_pause parameter.");
        }
        last_pause_state = pause;
    } else {
        ROS_ERROR("Failed to send command to rosbag process. Result: %d", result);
    }
}

void PdpLogEditorPanel::setInitialButtonStates() {
    // 只有Load from JSON按钮可用，其他按钮禁用
    load_button_->setEnabled(true);
    save_button_->setEnabled(false);
    manual_capture_button_->setEnabled(false);
    undo_button_->setEnabled(false);
    reset_button_->setEnabled(false);
    
    // 时间编辑框也禁用
    start_time_spinbox_->setEnabled(false);
    takeover_time_spinbox_->setEnabled(false);
    event_time_spinbox_->setEnabled(false);
    end_time_spinbox_->setEnabled(false);
    vehicle_config_edit_->setEnabled(false);
    
    // 更新状态提示
    status_label_->setText("请先点击【Load from JSON】加载配置文件");
    status_label_->setStyleSheet("font-weight: bold; color: orange;");
}

void PdpLogEditorPanel::enableAllButtons() {
    // 启用所有按钮和编辑框
    save_button_->setEnabled(true);
    manual_capture_button_->setEnabled(true);
    undo_button_->setEnabled(true);
    reset_button_->setEnabled(true);
    
    start_time_spinbox_->setEnabled(true);
    takeover_time_spinbox_->setEnabled(true);
    event_time_spinbox_->setEnabled(true);
    end_time_spinbox_->setEnabled(true);
    vehicle_config_edit_->setEnabled(true);
    
    // 恢复正常状态提示样式
    status_label_->setStyleSheet("font-weight: bold; color: blue;");
}

} // namespace pdp_log_editor
PLUGINLIB_EXPORT_CLASS(pdp_log_editor::PdpLogEditorPanel, rviz::Panel)