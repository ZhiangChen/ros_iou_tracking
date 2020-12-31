# ros_iou_tracking
A ROS package that applies multiple object tracking (MOT) to YOLO detection

## Requirements
#### Realtime object detector  
[leggedrobotics/darknet_ros](https://github.com/leggedrobotics/darknet_ros): A ROS package for YOLO detection

#### Multiple object tracker  
[abewley/sort](https://github.com/abewley/sort): simple online realtime tracker  

## Pipeline
1. train YOLO with your own data
2. launch darknet_ros with well-trained YOLO
3. launch ros_sort

## Semi-automatic annotation
1. train YOLO with a small set of annotations
2. automate annotation augmentation using offline object tracking
3. correct automatic annotations if necessary
4. retrain YOLO with previous and augmented data

## Related work
#### Mutiple object tracking
1. [bochinski/iou-tracker](https://github.com/bochinski/iou-tracker)  
[The original IoU tracker](http://elvera.nue.tu-berlin.de/files/1517Bochinski2017.pdf) is also using IoU information. However, it does not have Kalman filter (and Hungarian algorithm) for tracking, and only uses IoU for target association. It has several strict requirements:  
(1) there should be no detection gaps among frames; otherwise the tracked target will be removed.  
(2) the frame rate should be high enough so the IoU thresholds are guaranteed between successive frames.  
[The extended work](http://elvera.nue.tu-berlin.de/files/1547Bochinski2018.pdf) allows detection gaps by allowing bounding boxes that do not have new detections stay at the last position for several frames. Not only does it fill the detection gaps forward, it also does so backwards, which makes it less ideal for realtime tracking. Also, compared with SORT, predicting bounding box using Kalman filter that has considered the bounding box velocity is a better appoach to having undetected bounding box stay at the last position, when the camera motion is smooth.  

## Acknowledgement
[sort.py] is from [abewley/sort](https://github.com/abewley/sort)
