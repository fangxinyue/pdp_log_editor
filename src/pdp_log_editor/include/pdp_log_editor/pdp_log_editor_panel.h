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
    void onPublish();
    void onSaveToJson();
    void onLoadFromJson();
    void onManualTimeCapture();  // 新增：手动时间捕获按钮
    void onSyncTime();           // 新增：时间同步按钮
    void onTimelineChanged(int value);     // 新增：时间轴滑动槽函数
    void onTimestampClicked(int timestamp_type);  // 新增：时间戳点击槽函数
    void onStartTimeChanged(double value);    // 新增：开始时间编辑槽函数
    void onTakeoverTimeChanged(double value); // 新增：接管时间编辑槽函数
    void onEventTimeChanged(double value);    // 新增：事件时间编辑槽函数
    void onEndTimeChanged(double value);      // 新增：结束时间编辑槽函数
    void onPlayPause();                       // 新增：播放/暂停控制
    void onStop();                           // 新增：停止播放
    void onSpeedChanged(double speed);       // 新增：播放速度变化

private:
    // ROS 通信
    ros::Subscriber click_update_sub_;
    ros::Publisher control_cmd_pub_;
    ros::Publisher annotation_result_pub_;
    ros::Publisher click_event_pub_;  // 新增：用于手动时间发布

    // UI 元素
    QLabel* status_label_;
    QDoubleSpinBox* start_time_spinbox_;     // 修改：可编辑的开始时间(绿色)
    QDoubleSpinBox* takeover_time_spinbox_;  // 修改：可编辑的接管时间(黄色)
    QDoubleSpinBox* event_time_spinbox_;     // 修改：可编辑的事件时间(橙色)
    QDoubleSpinBox* end_time_spinbox_;       // 修改：可编辑的结束时间(红色)
    QPushButton* undo_button_;
    QPushButton* reset_button_;
    QPushButton* publish_button_;
    QPushButton* save_button_;
    QPushButton* load_button_;
    QPushButton* manual_capture_button_;  // 新增：手动时间捕获按钮
    QPushButton* sync_time_button_;       // 新增：时间同步按钮
    TimelineWidget* timeline_widget_;     // 新增：时间轴组件
    QSlider* timeline_slider_;            // 新增：时间轴滑动条（进度条）
    QLabel* current_time_label_;          // 新增：当前时间显示
    QPushButton* play_pause_button_;      // 新增：播放/暂停按钮
    QPushButton* stop_button_;            // 新增：停止按钮
    QDoubleSpinBox* speed_spinbox_;       // 新增：播放速度控制
    
    // 内部数据存储
    pdp_log_editor::PdpLogAnnotation current_annotation_;
    int clicks_done_;
    int manual_clicks_done_;  // 新增：手动点击计数器
    bool use_bag_time_;       // 新增：是否使用bag时间标志
    ros::Duration bag_time_offset_; // 新增：bag时间偏移量
    
    // 时间轴相关数据
    double timeline_start_time_;  // 时间轴开始时间
    double timeline_end_time_;    // 时间轴结束时间
    double current_timeline_time_; // 当前时间轴时间
    bool timeline_dragging_;      // 是否正在拖拽时间轴
    
    // 播放控制相关
    bool is_playing_;             // 是否正在播放
    double playback_speed_;       // 播放速度
    ros::Subscriber clock_sub_;   // /clock话题订阅器
    QTimer* timeline_update_timer_; // 时间轴更新定时器
    int user_operation_delay_;    // 用户操作延迟保护计数器（防止自动状态同步干扰）
    
    // rosbag控制服务客户端
    ros::ServiceClient play_service_;
    ros::ServiceClient pause_service_;

    // ROS回调和辅助函数
    void clickUpdateCallback(const pdp_log_editor::PdpLogAnnotation::ConstPtr& msg);
    void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg);  // 新增：/clock话题回调
    void updateTimelineDisplay();                                   // 新增：更新时间轴显示
    void updateUI();
    void resetManualCapture();  // 新增：重置手动捕获状态
    ros::Time getCurrentTime(); // 新增：获取当前时间（考虑同步模式）
    void initializeTimeline();  // 新增：初始化时间轴
    void updateTimelinePosition(); // 新增：更新时间轴位置
    double timestampToSliderValue(double timestamp); // 新增：时间戳转滑动条值
    double sliderValueToTimestamp(int slider_value); // 新增：滑动条值转时间戳
    void publishPlaybackCommand(const std::string& command); // 新增：发布播放控制命令
    void checkRosbagStatus(); // 新增：检查rosbag状态
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