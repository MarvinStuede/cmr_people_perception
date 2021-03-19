# Cluster Polygon
Package for the generation of polygons in RVIZ, which is used for example for the annotation of point clouds.

## Launch
To launch the node
```bash
roslaunch cluster_polygon cluster_polygon.launch
```

## Usage
After the launcher has been started, different topics must first be visualized in RVIZ. An example of all topics are in launcher/example.rviz.

**Topics**:

- /basic_control/update
- /cluster_polygon
- /cluster_polygon2
- /cluster_polygon3
- /cluster_polygon4
- /map

A total of four polygons can be created. A point of a polygon is created in RVIZ with "Publish Point" at the top of the bar. At least 3 points must be created for a polygon. With left-click on the points the position can be moved. With right-click a menu opens. In the menu several options can be executed:

- Delete -> Deletes the selected point
- Insert -> Adds new point to the left or right of the selected point
- Save -> Saves polygon to yaml file
- New -> Deletes all points of the polygon
- Change Polygon -> Change to another polygon
- Add Polygon -> Adds new polygon
