# ros_iou_tracking
A ROS package that applies multiple object tracking (MOT) to YOLO detection

## Requirements
#### Realtime object detector  
[leggedrobotics/darknet_ros](https://github.com/leggedrobotics/darknet_ros): A ROS package for YOLO detection

#### Multiple objector tracker  
Candidates:
- [abewley/sort](https://github.com/abewley/sort): simple online realtime tracker  
- [bochinski/iou-tracker](https://github.com/bochinski/iou-tracker)

## Pipeline
1. train YOLO with your own data
2. data augmentation using offline object tracking
3. retrain YOLO with augmented data
4. launch darknet_ros with well-trained YOLO
5. launch iou_tracker
