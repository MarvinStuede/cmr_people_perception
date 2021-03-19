# 3D Object Detector

This package includes a 3D object detector using a Support Vector Machine. The code is heavily based on [this repository](https://github.com/LCAS/online_learning).

## Launch
To start object detection using point cloud library (slow, computationally intensive, more accurate)
```bash
roslaunch object3d_detector object3d_detector.launch
```
Requires the pointcloud of the velodyne (/velodyne_points).
Detected persons are published under:

- **/object3d_detector/poses** Pose of the person

- **/object3d_detector/markers** Bounding box of the person

- **/spencer/perception_internal/detected_persons/velodyne** Detected person for Spencer


To start object detection using depth_clustering (fast, low computation, not so accurate)
```bash
roslaunch object3d_detector object3d_detector_fast.launch
```
Requires the pointcloud of the velodyne (/velodyne_points).
Detected persons are published under:

- **/object3d_detector/poses** Pose of the person

- **/object3d_detector/markers** Bounding box of the person

- **/spencer/perception_internal/detected_persons/velodyne** Detected person for Spencer


To save pointcloud clusters to train SVM (before that you have to adjust the folder path)
```bash
roslaunch object3d_detector cluster_node.launch
roslaunch cluster_polygon cluster_polygon.launch
```
Required:

- **/velodyne_points** The pointcloud of the velodyne

- **/cluster_polygon** Spanned polygon where clusters are filtered out (Created using "Publish Point" in RVIZ).

Returns:

- **/array_cluster** Saved pointcloud.


To control the stored pointcloud clusters
```bash
roslaunch object3d_detector show_pcd.launch
```
Before that, the folder with the saved .pcd files must be adjusted.

Operation by keyboard:

- **q** For more information about the Pointcloud. Cloud can be rotated in the window or aligned using r. Press q again to return.

- **a** View previous cloud

- **z** delete cloud

- **other keys** next cloud


To train the SVM
```bash
rosrun object3d_detector training_ob_multi
rosrun object3d_detector training_ob_multi_fast
```
Before that the folder with the saved .pcd files must be adjusted. Needed are clusters with persons and objects. The first node learns a SVM with 6 features for the object3d_detector. The second node learns a SVM with 5 features for the object3d_detector_fast. The node must be started in the folder `.../training`. The models have to be re-integrated in the respective launcher. For this the .model and .range file is needed.
