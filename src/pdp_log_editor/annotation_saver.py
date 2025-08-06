#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
from pdp_log_editor.msg import PdpLogAnnotation

OUTPUT_FILE = 'annotations.yaml'

def annotation_callback(msg):
    """
    接收到PdpLogAnnotation消息时的回调函数
    """
    rospy.loginfo("Received new annotation for log: %s", msg.source_log_name)

    # 将消息数据转换为字典格式
    annotation_data = {
        'source_log': msg.source_log_name,
        'annotation_timestamp_utc': msg.header.stamp.to_sec(),
        'events': {
            'scene_start': msg.scene_start_time.to_sec(),
            'takeover': msg.takeover_time.to_sec(),
            'event': msg.event_time.to_sec(),
            'scene_end': msg.scene_end_time.to_sec()
        }
    }

    # 读取现有文件内容
    try:
        with open(OUTPUT_FILE, 'r') as f:
            data = yaml.safe_load(f)
            if not isinstance(data, list):
                data = []
    except (IOError, yaml.YAMLError):
        data = []
        
    # 追加新数据并写回文件
    data.append(annotation_data)
    with open(OUTPUT_FILE, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    
    rospy.loginfo("Annotation saved to %s", OUTPUT_FILE)

def main():
    rospy.init_node('annotation_saver', anonymous=True)
    
    # 订阅标注结果话题
    rospy.Subscriber('/pdp_log_editor/annotation_result', PdpLogAnnotation, annotation_callback)
    
    rospy.loginfo("Annotation saver node started, waiting for annotations...")
    rospy.spin()

if __name__ == '__main__':
    main()