#include "pdp_log_editor/pdp_log_editor_tool.h"
#include <visualization_msgs/Marker.h>
#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <pluginlib/class_list_macros.h>

namespace pdp_log_editor {

PdpLogEditorTool::PdpLogEditorTool() : current_state_(AnnotationState::AWAITING_SCENE_START) {}

// 这是解决问题的关键实现！
PdpLogEditorTool::~PdpLogEditorTool() {}

void PdpLogEditorTool::onInitialize() {
    rviz::Tool::onInitialize();
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/pdp_log_editor/markers", 10);
    click_event_pub_ = nh_.advertise<pdp_log_editor::PdpLogAnnotation>("/pdp_log_editor/internal/click_update", 1);
    control_cmd_sub_ = nh_.subscribe("/pdp_log_editor/internal/control_cmd", 1, &PdpLogEditorTool::controlCmdCallback, this);
    
    // 添加注释：使用 ros::Time::now() 会自动处理仿真时间
    // 当播放 bag 文件时，如果设置了 use_sim_time=true，
    // ros::Time::now() 会返回 bag 文件的时间戳而不是系统时间
}

void PdpLogEditorTool::activate() { resetAnnotation(); }
void PdpLogEditorTool::deactivate() {}

int PdpLogEditorTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
    if (event.leftDown() && current_state_ != ANNOTATION_COMPLETE) {
        Ogre::Vector3 position;
        if (context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, position)) {
            timestamps_.push_back(ros::Time::now());
            placeVisualMarker(position, current_state_);
            current_state_ = static_cast<AnnotationState>(timestamps_.size());
            
            pdp_log_editor::PdpLogAnnotation msg;
            if (timestamps_.size() > 0) msg.scene_start_time = timestamps_[0];
            if (timestamps_.size() > 1) msg.takeover_time = timestamps_[1];
            if (timestamps_.size() > 2) msg.event_time = timestamps_[2];
            if (timestamps_.size() > 3) msg.scene_end_time = timestamps_[3];
            click_event_pub_.publish(msg);
        }
        return Render;
    }
    return rviz::Tool::processMouseEvent(event);
}

void PdpLogEditorTool::controlCmdCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "RESET") resetAnnotation();
    else if (msg->data == "UNDO") undoLastStep();
}

void PdpLogEditorTool::resetAnnotation() {
    current_state_ = AnnotationState::AWAITING_SCENE_START;
    timestamps_.clear();
    marker_ids_.clear();
    visualization_msgs::Marker marker;
    marker.header.frame_id = context_->getFixedFrame().toStdString();
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_pub_.publish(marker);
    pdp_log_editor::PdpLogAnnotation msg;
    click_event_pub_.publish(msg);
}

void PdpLogEditorTool::undoLastStep() {
    if (!timestamps_.empty()) {
        timestamps_.pop_back();
        if (!marker_ids_.empty()) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = context_->getFixedFrame().toStdString();
            marker.ns = "pdp_log_editor";
            marker.id = marker_ids_.back();
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub_.publish(marker);
            marker_ids_.pop_back();
        }
        current_state_ = static_cast<AnnotationState>(timestamps_.size());
        pdp_log_editor::PdpLogAnnotation msg;
        if (timestamps_.size() > 0) msg.scene_start_time = timestamps_[0];
        if (timestamps_.size() > 1) msg.takeover_time = timestamps_[1];
        if (timestamps_.size() > 2) msg.event_time = timestamps_[2];
        click_event_pub_.publish(msg);
    }
}

void PdpLogEditorTool::placeVisualMarker(const Ogre::Vector3& position, AnnotationState state) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = context_->getFixedFrame().toStdString();
    marker.header.stamp = ros::Time::now();
    marker.ns = "pdp_log_editor";
    marker.id = static_cast<int>(state);
    marker_ids_.push_back(marker.id);
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = position.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5; marker.scale.y = 0.5; marker.scale.z = 0.5;
    marker.color.a = 1.0;
    switch (state) {
        case AWAITING_SCENE_START: marker.color.r = 1.0; break;
        case AWAITING_TAKEOVER:    marker.color.g = 1.0; break;
        case AWAITING_EVENT:       marker.color.b = 1.0; break;
        case AWAITING_SCENE_END:   marker.color.r = 1.0; marker.color.g = 1.0; break;
        default: break;
    }
    marker_pub_.publish(marker);
}

int PdpLogEditorTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
  return rviz::Tool::processKeyEvent(event, panel);
}

void PdpLogEditorTool::load(const rviz::Config& config)
{
  rviz::Tool::load(config);
}

void PdpLogEditorTool::save(rviz::Config config) const
{
  rviz::Tool::save(config);
}

} // namespace pdp_log_editor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pdp_log_editor::PdpLogEditorTool, rviz::Tool)