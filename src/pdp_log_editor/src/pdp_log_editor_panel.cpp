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
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGridLayout>
#include <ros/ros.h>


namespace pdp_log_editor {

PdpLogEditorPanel::PdpLogEditorPanel(QWidget* parent) : rviz::Panel(parent), clicks_done_(0), manual_clicks_done_(0), use_bag_time_(false) {
    auto* layout = new QVBoxLayout(this);

    status_label_ = new QLabel("请在3D视图中激活工具并开始标注...");
    status_label_->setStyleSheet("font-weight: bold; color: blue;");
    layout->addWidget(status_label_);

    auto* grid_layout = new QGridLayout();
    grid_layout->addWidget(new QLabel("场景开始:"), 0, 0);
    start_time_label_ = new QLabel("N/A");
    grid_layout->addWidget(start_time_label_, 0, 1);

    grid_layout->addWidget(new QLabel("接管时间:"), 1, 0);
    takeover_time_label_ = new QLabel("N/A");
    grid_layout->addWidget(takeover_time_label_, 1, 1);

    grid_layout->addWidget(new QLabel("事件时间:"), 2, 0);
    event_time_label_ = new QLabel("N/A");
    grid_layout->addWidget(event_time_label_, 2, 1);

    grid_layout->addWidget(new QLabel("场景结束:"), 3, 0);
    end_time_label_ = new QLabel("N/A");
    grid_layout->addWidget(end_time_label_, 3, 1);

    layout->addLayout(grid_layout);

    undo_button_ = new QPushButton("撤销上一步");
    reset_button_ = new QPushButton("全部重置");
    publish_button_ = new QPushButton("发布标注结果");
    layout->addWidget(undo_button_);
    layout->addWidget(reset_button_);
    layout->addWidget(publish_button_);

    save_button_ = new QPushButton("Save to JSON");
    load_button_ = new QPushButton("Load from JSON");
    manual_capture_button_ = new QPushButton("手动获取时间 (0/4)");
    manual_capture_button_->setStyleSheet("font-weight: bold; color: green;");
    sync_time_button_ = new QPushButton("使用ROS时间");
    sync_time_button_->setStyleSheet("font-weight: bold; color: orange;");
    
    layout->addWidget(save_button_);
    layout->addWidget(load_button_);
    layout->addWidget(manual_capture_button_);
    layout->addWidget(sync_time_button_);

    connect(undo_button_, &QPushButton::clicked, this, &PdpLogEditorPanel::onUndo);
    connect(reset_button_, &QPushButton::clicked, this, &PdpLogEditorPanel::onReset);
    connect(publish_button_, &QPushButton::clicked, this, &PdpLogEditorPanel::onPublish);
    connect(save_button_, &QPushButton::clicked, this, &PdpLogEditorPanel::onSaveToJson);
    connect(load_button_, &QPushButton::clicked, this, &PdpLogEditorPanel::onLoadFromJson);
    connect(manual_capture_button_, &QPushButton::clicked, this, &PdpLogEditorPanel::onManualTimeCapture);
    connect(sync_time_button_, &QPushButton::clicked, this, &PdpLogEditorPanel::onSyncTime);

    setLayout(layout);
    updateUI();
}

PdpLogEditorPanel::~PdpLogEditorPanel() {}

void PdpLogEditorPanel::onInitialize() {
    rviz::Panel::onInitialize();
    ros::NodeHandle nh;
    click_update_sub_ = nh.subscribe("/pdp_log_editor/internal/click_update", 1, &PdpLogEditorPanel::clickUpdateCallback, this);
    control_cmd_pub_ = nh.advertise<std_msgs::String>("/pdp_log_editor/internal/control_cmd", 1);
    annotation_result_pub_ = nh.advertise<pdp_log_editor::PdpLogAnnotation>("/pdp_log_editor/annotation_result", 1, true);
    click_event_pub_ = nh.advertise<pdp_log_editor::PdpLogAnnotation>("/pdp_log_editor/internal/click_update", 1);  // 新增：用于手动时间发布
}

void PdpLogEditorPanel::onUndo() {
    // 优先处理手动点击的撤销
    if (manual_clicks_done_ > 0) {
        manual_clicks_done_--;
        
        // 清除对应的时间戳
        switch (manual_clicks_done_) {
            case 0:
                current_annotation_.scene_start_time = ros::Time(0);
                status_label_->setText("已撤销场景开始时间");
                break;
            case 1:
                current_annotation_.takeover_time = ros::Time(0);
                status_label_->setText("已撤销接管时间");
                break;
            case 2:
                current_annotation_.event_time = ros::Time(0);
                status_label_->setText("已撤销事件时间");
                break;
            case 3:
                current_annotation_.scene_end_time = ros::Time(0);
                status_label_->setText("已撤销场景结束时间");
                break;
        }
        
        // 更新按钮状态
        manual_capture_button_->setText(QString("手动获取时间 (%1/4)").arg(manual_clicks_done_));
        manual_capture_button_->setStyleSheet("font-weight: bold; color: green;");
        
        // 同步点击计数
        clicks_done_ = manual_clicks_done_;
        
        updateUI();
        
        // 发布更新
        click_event_pub_.publish(current_annotation_);
    } else {
        // 如果没有手动点击，则发送撤销命令给工具
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
    if (manual_clicks_done_ >= 4) {
        status_label_->setText("已完成4次时间捕获，请先重置！");
        return;
    }
    
    ros::Time current_time = getCurrentTime();
    
    // 根据点击次数设置对应的时间
    switch (manual_clicks_done_) {
        case 0:
            current_annotation_.scene_start_time = current_time;
            status_label_->setText(QString("已捕获场景开始时间: %1 (%2)").arg(current_time.toSec(), 0, 'f', 2).arg(use_bag_time_ ? "Bag时间" : "ROS时间"));
            break;
        case 1:
            current_annotation_.takeover_time = current_time;
            status_label_->setText(QString("已捕获接管时间: %1 (%2)").arg(current_time.toSec(), 0, 'f', 2).arg(use_bag_time_ ? "Bag时间" : "ROS时间"));
            break;
        case 2:
            current_annotation_.event_time = current_time;
            status_label_->setText(QString("已捕获事件时间: %1 (%2)").arg(current_time.toSec(), 0, 'f', 2).arg(use_bag_time_ ? "Bag时间" : "ROS时间"));
            break;
        case 3:
            current_annotation_.scene_end_time = current_time;
            status_label_->setText(QString("已捕获场景结束时间: %1 (%2)").arg(current_time.toSec(), 0, 'f', 2).arg(use_bag_time_ ? "Bag时间" : "ROS时间"));
            break;
    }
    
    manual_clicks_done_++;
    clicks_done_ = manual_clicks_done_;  // 同步点击计数
    
    // 更新按钮文本
    manual_capture_button_->setText(QString("手动获取时间 (%1/4)").arg(manual_clicks_done_));
    
    if (manual_clicks_done_ >= 4) {
        manual_capture_button_->setText("手动获取完成 (4/4)");
        manual_capture_button_->setStyleSheet("font-weight: bold; color: blue;");
        status_label_->setText("手动时间捕获完成，可点击发布！");
    }
    
    updateUI();
    
    // 发布更新给其他组件
    click_event_pub_.publish(current_annotation_);
}

void PdpLogEditorPanel::onPublish() {
    if (clicks_done_ == 4) {
        current_annotation_.header.stamp = ros::Time::now();
        current_annotation_.header.frame_id = "map";
        annotation_result_pub_.publish(current_annotation_);
        status_label_->setText("标注结果已发布！");
    } else {
        status_label_->setText("标注未完成，无法发布！");
    }
}

void PdpLogEditorPanel::clickUpdateCallback(const pdp_log_editor::PdpLogAnnotation::ConstPtr& msg) {
    current_annotation_ = *msg;
    clicks_done_ = 0;
    if (msg->scene_start_time.toSec() > 0) clicks_done_++;
    if (msg->takeover_time.toSec() > 0) clicks_done_++;
    if (msg->event_time.toSec() > 0) clicks_done_++;
    if (msg->scene_end_time.toSec() > 0) clicks_done_++;
    updateUI();
}

void PdpLogEditorPanel::updateUI() {
    start_time_label_->setText(current_annotation_.scene_start_time.toSec() > 0 ? QString::number(current_annotation_.scene_start_time.toSec(), 'f', 2) : "N/A");
    takeover_time_label_->setText(current_annotation_.takeover_time.toSec() > 0 ? QString::number(current_annotation_.takeover_time.toSec(), 'f', 2) : "N/A");
    event_time_label_->setText(current_annotation_.event_time.toSec() > 0 ? QString::number(current_annotation_.event_time.toSec(), 'f', 2) : "N/A");
    end_time_label_->setText(current_annotation_.scene_end_time.toSec() > 0 ? QString::number(current_annotation_.scene_end_time.toSec(), 'f', 2) : "N/A");

    switch (clicks_done_) {
        case 0: status_label_->setText("请点击以标注【场景开始时间】"); break;
        case 1: status_label_->setText("请点击以标注【接管时间】"); break;
        case 2: status_label_->setText("请点击以标注【事件时间】"); break;
        case 3: status_label_->setText("请点击以标注【场景结束时间】"); break;
        case 4: status_label_->setText("标注已完成，可点击发布"); break;
        default: break;
    }
}

void PdpLogEditorPanel::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

void PdpLogEditorPanel::load(const rviz::Config& config) {
    rviz::Panel::load(config);
}

void PdpLogEditorPanel::onSaveToJson() {
    if (clicks_done_ < 4) {
        status_label_->setText("标注未完成，无法保存！");
        return;
    }

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
        
        if (!j.contains("vehicle_config")) {
            j["vehicle_config"] = "JLBJA47708D";
        }
        
        if (!j.contains("db.sqlite")) {
            j["db.sqlite"] = "https://autocar-mogosim-1255510688.cos.ap-beijing.myqcloud.com/hadmap/bj-V2.9.8.sqlite";
        }

        std::ofstream out(file_path.toStdString());
        out << j.dump(4);
        out.close();

        status_label_->setText("标注已保存到JSON文件！");
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

        // 计算已完成的点击次数
        clicks_done_ = 0;
        manual_clicks_done_ = 0;
        if (current_annotation_.scene_start_time.toSec() > 0) { clicks_done_++; manual_clicks_done_++; }
        if (current_annotation_.takeover_time.toSec() > 0) { clicks_done_++; manual_clicks_done_++; }
        if (current_annotation_.event_time.toSec() > 0) { clicks_done_++; manual_clicks_done_++; }
        if (current_annotation_.scene_end_time.toSec() > 0) { clicks_done_++; manual_clicks_done_++; }
        
        // 更新手动捕获按钮状态
        if (manual_clicks_done_ >= 4) {
            manual_capture_button_->setText("手动获取完成 (4/4)");
            manual_capture_button_->setStyleSheet("font-weight: bold; color: blue;");
        } else {
            manual_capture_button_->setText(QString("手动获取时间 (%1/4)").arg(manual_clicks_done_));
            manual_capture_button_->setStyleSheet("font-weight: bold; color: green;");
        }
        
        updateUI();
        status_label_->setText(QString("标注已从JSON文件加载！(%1/4 完成)").arg(clicks_done_));
    } catch (const std::exception& e) {
        status_label_->setText(QString("加载失败: ") + e.what());
    }
}

void PdpLogEditorPanel::resetManualCapture() {
    manual_clicks_done_ = 0;
    manual_capture_button_->setText("手动获取时间 (0/4)");
    manual_capture_button_->setStyleSheet("font-weight: bold; color: green;");
    
    // 清除所有手动捕获的时间
    current_annotation_.scene_start_time = ros::Time(0);
    current_annotation_.takeover_time = ros::Time(0);
    current_annotation_.event_time = ros::Time(0);
    current_annotation_.scene_end_time = ros::Time(0);
    
    clicks_done_ = 0;
    updateUI();
    
    // 发布清空的状态
    click_event_pub_.publish(current_annotation_);
}

void PdpLogEditorPanel::onSyncTime() {
    use_bag_time_ = !use_bag_time_;
    
    if (use_bag_time_) {
        // 计算bag时间偏移
        ros::Time ros_now = ros::Time::now();
        ros::Time bag_now(0);
        
        // 尝试从/clock话题获取仿真时间
        try {
            rosgraph_msgs::ClockConstPtr clock_msg = ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", ros::Duration(1.0));
            if (clock_msg) {
                bag_now = clock_msg->clock;
                bag_time_offset_ = bag_now - ros_now;
                sync_time_button_->setText(QString("使用Bag时间 (偏移: %1s)").arg(bag_time_offset_.toSec(), 0, 'f', 2));
                sync_time_button_->setStyleSheet("font-weight: bold; color: blue;");
                status_label_->setText("已切换到Bag时间模式");
            } else {
                throw std::runtime_error("无法获取/clock话题");
            }
        } catch (...) {
            // 如果无法获取/clock，假设没有时间偏移
            bag_time_offset_ = ros::Duration(0);
            sync_time_button_->setText("使用Bag时间 (无偏移)");
            sync_time_button_->setStyleSheet("font-weight: bold; color: purple;");
            status_label_->setText("切换到Bag时间模式（未检测到/clock话题）");
        }
    } else {
        sync_time_button_->setText("使用ROS时间");
        sync_time_button_->setStyleSheet("font-weight: bold; color: orange;");
        status_label_->setText("已切换到ROS系统时间模式");
    }
}

ros::Time PdpLogEditorPanel::getCurrentTime() {
    if (use_bag_time_) {
        // 尝试获取当前的仿真时间
        try {
            rosgraph_msgs::ClockConstPtr clock_msg = ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", ros::Duration(0.1));
            if (clock_msg) {
                return clock_msg->clock;
            }
        } catch (...) {
            // 如果无法获取/clock，使用ROS时间加偏移
        }
        return ros::Time::now() + bag_time_offset_;
    } else {
        return ros::Time::now();
    }
}

} // namespace pdp_log_editor
PLUGINLIB_EXPORT_CLASS(pdp_log_editor::PdpLogEditorPanel, rviz::Panel)