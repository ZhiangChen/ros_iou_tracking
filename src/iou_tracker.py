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
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import copy
import numpy as np
import cv2
from sort import Sort
from sort import associate_detections_to_trackers

class IoUTracker(object):
    def __init__(self, max_age=50, min_hits=50, iou_threshold=0.3, display=False):
        """
        ROS IoU Tracker
        :param max_age: Maximum number of frames to keep alive a track without associated detections.
        :param min_hits: Minimum number of associated detections before track is initialised.
        :param iou_threshold: Minimum IOU for match.
        """
        self.iou_threshold = iou_threshold
        self.display = display
        self.bridge = CvBridge()
        self.tracked_img_pub = rospy.Publisher("/iou_tracker/detection_image", Image, queue_size=1)
        self.bboxes = []
        self.bboxes_msg = BoundingBoxes()
        self.traces = dict()
        self.mot_tracker = Sort(max_age=max_age,
                                min_hits=min_hits,
                                iou_threshold=iou_threshold)  # create instance of the SORT tracker
        self.image = np.zeros(1)
        self.raw_image_sub = rospy.Subscriber('/darknet_ros/detection_image', Image, self.__raw_image_callback, queue_size=1)

        self.bbox_pub = rospy.Publisher("/iou_tracker/bounding_boxes_drop", BoundingBoxes, queue_size=1)
        self.bbox_nn_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.__bbox_nn_callback,
                                       queue_size=1)
        rospy.loginfo("iou_tracker has been initialized!")


    def __bbox_nn_callback(self, data):
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
        # 1. get SORT bounding boxes
        new_bboxes_sort = []
        for bbox in new_bboxes:
            new_bboxes_sort.append(np.asarray((bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax, bbox.probability)))

        new_bboxes_sort = np.asarray(new_bboxes_sort)
        # 2. update tracker
        trackers = self.mot_tracker.update(new_bboxes_sort)

        matched, _, _ = associate_detections_to_trackers(new_bboxes_sort, trackers, 0.3)

        # 3. update current bounding boxes & extract tracking trace
        # 1). id is the id from tracker;
        # 2). class is the latest class from detection;
        # 3). probability is the latest probability from detection;
        self.bboxes = []
        ids = []
        if trackers.shape[0] != 0:
            for tracker in trackers:
                bbox = BoundingBox()
                bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax = tracker[0], tracker[1], tracker[2], tracker[3]
                bbox.id = tracker[4]
                self.bboxes.append(bbox)
                id = int(bbox.id)
                center = (int((bbox.xmin + bbox.xmax)/2.), int((bbox.ymin + bbox.ymax)/2.))
                if self.traces.get(id) == None:
                    self.traces[id] = [center]
                else:
                    self.traces[id].append(center)
                ids.append(id)

        if matched.shape[0] != 0:
            for pair in matched:
                original_id = pair[0]
                new_id = pair[1]
                original_bbox = new_bboxes[original_id]
                self.bboxes[new_id].Class = original_bbox.Class
                self.bboxes[new_id].probability = original_bbox.probability

        del_ids = []
        for k in self.traces:
            if k not in ids:
                del_ids.append(k)

        for k in del_ids:
            del self.traces[k]

        # 4. publish current bounding boxes
        self.__publish_bbox()

        # 5. publish tracking image
        self.__publish_tracking_image(self.image, self.bboxes)


    def __publish_tracking_image(self, image, bboxes):
        Xs = copy.deepcopy(bboxes)
        if len(image.shape) <= 2:
            return
        for x in Xs:
            image = cv2.rectangle(image, (int(x.xmin), int(x.ymin)), (int(x.xmax), int(x.ymax)), (255, 255, 0), 2)

        for k in self.traces:
            trace = self.traces[k]
            if len(trace) >= 200:
                trace = trace[-200:]
                self.traces[k] = trace
            pts = copy.deepcopy(trace)
            pts.reverse()
            for i in range(1, len(pts)):
                # if either of the tracked points are None, ignore
                # them
                if pts[i - 1] is None or pts[i] is None:
                    continue
                # otherwise, compute the thickness of the line and
                # draw the connecting lines
                buffer = 32
                thickness = int(np.sqrt(buffer / float(i + 1)) * 2.5)
                cv2.line(image, pts[i - 1], pts[i], (255, 255, 0), thickness)

        image_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.tracked_img_pub.publish(image_msg)


    def __raw_image_callback(self, data):
        self.image_header = data.header
        raw_image = self.bridge.imgmsg_to_cv2(data).astype(np.uint8)
        if len(raw_image.shape) > 2:
            self.image = raw_image


    def __publish_bbox(self):
        if len(self.bboxes) != 0:
            if len(self.image.shape) > 2:
                self.bboxes_msg.header = self.image_header
                self.bboxes_msg.image_header = self.image_header
                self.bboxes_msg.bounding_boxes = copy.deepcopy(self.bboxes)
                self.bbox_pub.publish(self.bboxes_msg)

if __name__ == '__main__':
    rospy.init_node('iou_tracker', anonymous=False)
    iou_tracker = IoUTracker()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node killed!")

"""
for i in range(1, len(pts)):
    # if either of the tracked points are None, ignore
    # them
    if pts[i - 1] is None or pts[i] is None:
        continue
    # otherwise, compute the thickness of the line and
    # draw the connecting lines
    thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
    cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
"""