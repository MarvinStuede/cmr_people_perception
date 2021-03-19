# Guiding Utils

This package contains utility programs


## Launch
Median filter to calculate the position of persons
```bash
rosrun guiding_utils projection_darknet
```

Requires:

- `/darknet_ros/boundingbox` Bounding box from YOLO
- `.../image_raw` depth image of the camera or velodyne

Fusion of detections for Spencer
```bash
rosrun guiding_utils detection_com
```

Requires detections (optional):

- `/spencer/perception_internal/detected_persons/pre/velodyne`
- `/spencer/perception_internal/detected_persons/pre/velodyne_fast`
- `/spencer/perception_internal/detected_persons/pre/rgbd_front_top/upper_body`
- `/spencer/perception_internal/detected_persons/pre/rgbd_rear_top/upper_body`
- `/spencer/perception_internal/detected_persons/pre/laser_front`
