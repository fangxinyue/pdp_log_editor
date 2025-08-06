#ifndef PDP_LOG_EDITOR_PANEL_H
#define PDP_LOG_EDITOR_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include "pdp_log_editor/PdpLogAnnotation.h" // 包含自定义消息

// 前向声明Qt类
class QLabel;
class QPushButton;
class QVBoxLayout;

namespace pdp_log_editor {

class PdpLogEditorPanel : public rviz::Panel {
    Q_OBJECT

public:
    explicit PdpLogEditorPanel(QWidget* parent = nullptr);
    ~PdpLogEditorPanel() override;

    void onInitialize() override;
    void load(const rviz::Config& config) override;
    void save(rviz::Config config) const override;

protected Q_SLOTS:
    // 按钮点击事件的槽函数
    void onUndo();
    void onReset();
    void onPublish();
    void onSaveToJson();
    void onLoadFromJson();
    void onManualTimeCapture();  // 新增：手动时间捕获按钮

private:
    // ROS 通信
    ros::Subscriber click_update_sub_;
    ros::Publisher control_cmd_pub_;
    ros::Publisher annotation_result_pub_;
    ros::Publisher click_event_pub_;  // 新增：用于手动时间发布

    // UI 元素
    QLabel* status_label_;
    QLabel* start_time_label_;
    QLabel* takeover_time_label_;
    QLabel* event_time_label_;
    QLabel* end_time_label_;
    QPushButton* undo_button_;
    QPushButton* reset_button_;
    QPushButton* publish_button_;
    QPushButton* save_button_;
    QPushButton* load_button_;
    QPushButton* manual_capture_button_;  // 新增：手动时间捕获按钮
    
    // 内部数据存储
    pdp_log_editor::PdpLogAnnotation current_annotation_;
    int clicks_done_;
    int manual_clicks_done_;  // 新增：手动点击计数器

    // ROS回调和辅助函数
    void clickUpdateCallback(const pdp_log_editor::PdpLogAnnotation::ConstPtr& msg);
    void updateUI();
    void resetManualCapture();  // 新增：重置手动捕获状态
};

} // namespace pdp_log_editor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pdp_log_editor::PdpLogEditorPanel, rviz::Panel)

#endif // PDP_LOG_EDITOR_PANEL_H