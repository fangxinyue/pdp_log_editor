#ifndef PDP_LOG_EDITOR_PANEL_H
#define PDP_LOG_EDITOR_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <rosgraph_msgs/Clock.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include "pdp_log_editor/PdpLogAnnotation.h" // 包含自定义消息

// 前向声明Qt类
class QLabel;
class QPushButton;
class QVBoxLayout;
class QSlider;
class QHBoxLayout;
class QDoubleSpinBox;
class QTimer;
class QLineEdit; // 新增

namespace pdp_log_editor {

// 自定义时间轴Widget
class TimelineWidget;

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
    void onSaveToJson();
    void onLoadFromJson();
    void onLoadBagInfo();
    void onManualTimeCapture();
    void onTimelineChanged(int value);
    void onTimestampClicked(int timestamp_type);
    void onStartTimeChanged(double value);
    void onTakeoverTimeChanged(double value);
    void onEventTimeChanged(double value);
    void onEndTimeChanged(double value);

private:
    // ROS回调和辅助函数
    void checkSimPauseParam(const ros::TimerEvent&);
    void toggleRosbagPlayback(bool pause);
    void clickUpdateCallback(const pdp_log_editor::PdpLogAnnotation::ConstPtr& msg);
    void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg);
    void updateTimelineDisplay();
    void updateUI();
    void resetManualCapture();
    ros::Time getCurrentTime();
    void initializeTimeline();
    void updateTimelinePosition();
    double timestampToSliderValue(double timestamp);
    double sliderValueToTimestamp(int slider_value);

    // ROS 通信
    ros::NodeHandle nh_;
    ros::Subscriber click_update_sub_;
    ros::Publisher control_cmd_pub_;
    ros::Publisher annotation_result_pub_;
    ros::Publisher click_event_pub_;
    ros::Subscriber clock_sub_;
    ros::Timer param_check_timer_;

    // UI 元素
    QLabel* status_label_;
    QDoubleSpinBox* start_time_spinbox_;
    QDoubleSpinBox* takeover_time_spinbox_;
    QDoubleSpinBox* event_time_spinbox_;
    QDoubleSpinBox* end_time_spinbox_;
    QLineEdit* vehicle_config_edit_; // 新增
    QPushButton* undo_button_;
    QPushButton* reset_button_;
    QPushButton* save_button_;
    QPushButton* load_button_;
    QPushButton* load_bag_info_button_;
    QPushButton* manual_capture_button_;
    TimelineWidget* timeline_widget_;
    QSlider* timeline_slider_;
    QLabel* current_time_label_;
    QPushButton* play_pause_button_;
    QPushButton* stop_button_;
    QDoubleSpinBox* speed_spinbox_;
    QTimer* timeline_update_timer_;
    
    // 内部数据存储
    pdp_log_editor::PdpLogAnnotation current_annotation_;
    int clicks_done_;
    int manual_clicks_done_;
    bool use_bag_time_;
    ros::Duration bag_time_offset_;
    
    // 时间轴相关数据
    double timeline_start_time_;
    double timeline_end_time_;
    double current_timeline_time_;
    bool timeline_dragging_;
    
    // 播放控制相关
    bool is_playing_;
    double playback_speed_;
    bool last_sim_paused_state_;
    int user_operation_delay_;
};

// 自定义时间轴可视化Widget
class TimelineWidget : public QWidget {
    Q_OBJECT

public:
    explicit TimelineWidget(QWidget* parent = nullptr);
    
    void setTimeRange(double start_time, double end_time);
    void setTimestamps(double start, double takeover, double event, double end);
    void setCurrentTime(double current_time);

Q_SIGNALS:
    void timestampClicked(int timestamp_type); // 0:start, 1:takeover, 2:event, 3:end

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    double start_time_, end_time_;
    double timestamps_[4]; // start, takeover, event, end
    double current_time_;
    int widget_width_;
    int dragging_timestamp_; // -1: none, 0-3: timestamp index
    
    QColor timestamp_colors_[4]; // 不同时间戳的颜色
    
    double pixelToTime(int pixel_x);
    int timeToPixel(double time);
    int getTimestampAtPosition(int x, int y);
};

} // namespace pdp_log_editor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pdp_log_editor::PdpLogEditorPanel, rviz::Panel)

#endif // PDP_LOG_EDITOR_PANEL_H