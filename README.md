# yolo_object_tracker
A ROS package that applies object tracking to YOLO detection

## Requirements
1. [leggedrobotics/darknet_ros](https://github.com/leggedrobotics/darknet_ros): A ROS package for YOLO detection

2. [bochinski/iou-tracker](https://github.com/bochinski/iou-tracker): Real-time object tracker

## Pipeline
1. train YOLO with your own data
2. data augmentation using offline object tracking
3. retrain YOLO with augmented data
4. launch darknet_ros with well-trained YOLO
5. launch iou_tracker
