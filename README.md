# **cmr_people_perception**: Starters and packages for people detection and tracking
This repository contains packages to detect and track people with camera and lidar based detectors. It heavily relies on the [SPENCER Framework](https://github.com/spencer-project/spencer_people_tracking). Each package contains a separate README. See the `cmr_people_perception` package for instructions on how to start the framework.
### Packages

#### CMR People Perception
Includes launchers to run the framework

#### Cluster Polygon
Package for the generation of polygons in RVIZ, which is used for example for the annotation of point clouds.

#### Depth Clustering
Algorithm for clustering of a point cloud

#### Guiding Utils
This package contains util programs (median filter, fusion of detections)

#### sg_msgs
All required messages, services and actions are defined here

#### Object3D Detector
This package includes a 3D object detector using a Support Vector Machine

### Prerequisites and Installing
See general instructions [here](https://marvinstuede.github.io/Sobi/software/).
