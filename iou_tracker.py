#!/usr/bin/env python
"""
    ros_iou_tracking
    Copyright 2021 Zhiang Chen

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
"""

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from sort import Sort

class IoUTracker(object):
    def __init__(self, max_age=3, min_hits=3, iou_threshold=0.3):
        """
        ROS IoU Tracker
        :param max_age: Maximum number of frames to keep alive a track without associated detections.
        :param min_hits: Minimum number of associated detections before track is initialised.
        :param iou_threshold: Minimum IOU for match.
        """
        self.mot_tracker = Sort(max_age=max_age,
                                min_hits=min_hits,
                                iou_threshold=iou_threshold)  # create instance of the SORT tracker

        self.bbox_nn_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_nn_callback,
                                       queue_size=1)
        self.bbox_pub = rospy.Publisher("/iou_tracker/bounding_boxes", BoundingBoxes, queue_size=1)

    def bbox_nn_callback(self, data):
        """
        rosmsg info darknet_ros_msgs/BoundingBoxes
        float64 probability
        int64 xmin
        int64 ymin
        int64 xmax
        int64 ymax
        int16 id
        string Class
        :param data:
        :return:
        """
        new_bboxes = data.bounding_boxes  # new_bboxes is a list of darknet_ros_msgs.msg.BoundingBox
        # get SORT bounding boxes

        # update tracker. using get_matched argument in update() to support multi-class tracking

        # update current bounding boxes.
        # 1. id is the id from tracker;
        # 2. class is the latest class from detection;
        # 3. probability is the latest probability from detection.

        # publish current bounding boxes


if __name__ == '__main__':
    rospy.init_node('iou_tracker', anonymous=False)
    iou_tracker = IoUTracker()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node killed!")

