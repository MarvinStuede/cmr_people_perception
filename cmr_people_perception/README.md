# CMR People Perception
Package for launching the person recognition


## Launch
To start the detection and tracking framework:
```bash
roslaunch cmr_people_perception people_perception.launch
```

In the launch file, you can specify which detectors are to be activated. The choice is:

**RGBD detector**:

- Spencer Upperbody
- Spencer GroundHOG
- Darknet ROS (YOLO) (activated by default)
- Tensorflow

**3D Object Detector**:

- Detector with 6 features (enabled by default)
- Detector with 5 features (enabled by default)

**2D Object Detector**:

- Spencer leg detectors
- ROS leg detector
