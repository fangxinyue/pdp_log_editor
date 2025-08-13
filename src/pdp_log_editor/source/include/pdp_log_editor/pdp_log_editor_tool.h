#ifndef PDP_LOG_EDITOR_TOOL_H
#define PDP_LOG_EDITOR_TOOL_H

#include <ros/ros.h>
#include <rviz/tool.h>
#include <OgreVector3.h>
#include <vector>
#include "pdp_log_editor/PdpLogAnnotation.h" // 包含自定义消息
#include <std_msgs/String.h>

// 前向声明
namespace rviz {
class ViewportMouseEvent;
class RenderPanel;
}
class QKeyEvent;

namespace pdp_log_editor {

// 定义标注状态的枚举
enum AnnotationState {
    AWAITING_SCENE_START = 0,
    AWAITING_TAKEOVER = 1,
    AWAITING_EVENT = 2,
    AWAITING_SCENE_END = 3,
    ANNOTATION_COMPLETE = 4
};

class PdpLogEditorTool : public rviz::Tool {
    Q_OBJECT

public:
    PdpLogEditorTool();
    // 这是解决问题的关键声明！
    ~PdpLogEditorTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz::ViewportMouseEvent& event) override;
    int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

    void load(const rviz::Config& config) override;
    void save(rviz::Config config) const override;

private:
    void controlCmdCallback(const std_msgs::String::ConstPtr& msg);
    void resetAnnotation();
    void undoLastStep();
    void placeVisualMarker(const Ogre::Vector3& position, AnnotationState state);

    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Publisher click_event_pub_;
    ros::Subscriber control_cmd_sub_;

    AnnotationState current_state_;
    std::vector<ros::Time> timestamps_;
    std::vector<int> marker_ids_;
};

} // namespace pdp_log_editor

#endif // PDP_LOG_EDITOR_TOOL_H