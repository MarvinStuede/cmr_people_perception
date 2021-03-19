/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the GPL-style license found in the
LICENSE file in the root directory of this source tree.
*/
/*!
  *\file: cluster_node.cpp
  * \author: Alexander Petersen
  * \date: 21.11.19
  * \brief: ClusterNode Classe:
  * Class to extract Cluster from pointcloud
  */

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
// SVM
#include "svm.h"

#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


/**
  * A structure to define Features for classifcation
  * You can (un)comment for different use of Features.
  * !!If you change something, you have to change FEATURE_SIZE below as well!!
  */
typedef struct feature {
  /*** for visualization ***/
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  /*** for classification ***/
  int number_points;
  float min_distance;
  Eigen::Matrix3f covariance_3d;
  Eigen::Matrix3f moment_3d;
  // float partial_covariance_2d[9];
  // float histogram_main_2d[98];
  // float histogram_second_2d[45];
  float slice[20];
  float intensity[27];
} Feature;


static const int FEATURE_SIZE = 61;  /**< Dimension of Featurevector */


class ClusterNode {
private:
  /// Publishers and Subscribers
  ros::NodeHandle node_handle_;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber polygon_sub;
  ros::Publisher vis_pub;
  ros::Publisher vis_array_pub;


  bool print_fps_;
  std::string frame_id_;

  /// filtering pointcloud
  float z_limit_min_;
  float z_limit_max_;
  int cluster_size_min_;
  int cluster_size_max_;

  /// filter parameter cluster
  float vfilter_min_x_;
  float vfilter_max_x_;
  float vfilter_min_y_;
  float vfilter_max_y_;
  float vfilter_min_z_;
  float vfilter_max_z_;
  float cluster_min_z_;


  std::vector<Feature> features_;   /**< global featurevector for every pointcloud */

  bool human_size_limit_;
  //pcl::visualization::PCLVisualizer p_viewer;

  geometry_msgs::PolygonStamped polygon_points1;
  boost::mutex mutex;
  tf::TransformListener _listener;
  tf::StampedTransform transform;

public:
  /** Creating Clusternode Class
   * Initialising all subscribers, publishers and parameters
 */
  ClusterNode();
  ~ClusterNode();

  /// Callback for recieving Pointcloud
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2);

  /// Callback for recieving Polygon
  void polygoncallback(const geometry_msgs::PolygonStampedConstPtr& poly_msg);

  /** \brief Extract cluster.
    * \param pc Pointcloud
    *
    * This method extract Clusters from the given Pointcloud
    */
  void extractCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);

  /** \brief Extract features.
    * \param pc Pointcloud
    * \param f Featurevector
    * \param min Min. size of cluster
    * \param max Max. size of cluster
    * \param centroid Center of cluster
    *
    * This method calculate Features from given cluster
    */
  void extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f,
		      Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Vector4f &centroid);


};

ClusterNode::ClusterNode() {

  // Init subcriber
  point_cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, &ClusterNode::pointCloudCallback, this);
  polygon_sub =node_handle_.subscribe<geometry_msgs::PolygonStamped>("/cluster_polygon",1,&ClusterNode::polygoncallback,this);

  ros::NodeHandle private_nh("~");

  // Init publisher
  vis_pub = node_handle_.advertise<visualization_msgs::Marker>( "/marker", 10 );
  vis_array_pub = node_handle_.advertise<visualization_msgs::Marker>( "/array_cluster", 10 );



  /*** Parameters ***/
  private_nh.param<bool>("print_fps", print_fps_, false);
  private_nh.param<std::string>("frame_id", frame_id_, "velodyne");
  private_nh.param<float>("z_limit_min", z_limit_min_, -1.1);
  private_nh.param<float>("z_limit_max", z_limit_max_, 0.9);
  private_nh.param<int>("cluster_size_min", cluster_size_min_, 25);
  private_nh.param<int>("cluster_size_max", cluster_size_max_, 30000);

  private_nh.param<bool>("human_size_limit", human_size_limit_, false);

  private_nh.param<float>("vfilter_min_x", vfilter_min_x_, 0.1);
  private_nh.param<float>("vfilter_max_x", vfilter_max_x_, 1.3);
  private_nh.param<float>("vfilter_min_y", vfilter_min_y_, 0.1);
  private_nh.param<float>("vfilter_max_y", vfilter_max_y_, 1.3);
  private_nh.param<float>("vfilter_min_z", vfilter_min_z_, 0.1);
  private_nh.param<float>("vfilter_max_z", vfilter_max_z_, 2.0);
  private_nh.param<float>("cluster_min_z", cluster_min_z_, 2.0);

}

ClusterNode::~ClusterNode() {

}

void ClusterNode::polygoncallback(const geometry_msgs::PolygonStampedConstPtr &poly_msg)
{
  {

    boost::mutex::scoped_lock lock(mutex);
    polygon_points1=*poly_msg;   //saving polygon

  }


}

/** \brief InPolygon
  * \param nvert Size of polygon
  * \param vertx Vector of all x elements of polygon
  * \param verty Vector of all y elements of polygon
  * \param testx x element of point to check
  * \param testy y element of point to check
  *
  * This method is checking if a point is inside a give polygon.
  */
int InPolygon(int nvert, std::vector<double> vertx, std::vector<double> verty, double testx, double testy)
  {
      int i, j, c = 0;
      for (i = 0, j = nvert-1; i < nvert; j = i++)
      {
        if ( ((verty.at(i)>testy) != (verty.at(j)>testy)) && (testx < (vertx.at(j)-vertx.at(i)) * (testy-verty.at(i)) / (verty.at(j)-verty.at(i)) + vertx.at(i)) )
         { c = !c;
         }
      }
      return c;
  }


int frames; clock_t start_time; bool reset = true;//fps
void ClusterNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2) {
  if(print_fps_)if(reset){frames=0;start_time=clock();reset=false;}   //fps

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*ros_pc2, *pcl_pc);  // transforms pointcloud msgs to pcl format

  extractCluster(pcl_pc);

  if(print_fps_)if(++frames>10){std::cerr<<"[object3d_detector]: fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<", timestamp = "<<clock()/CLOCKS_PER_SEC<<std::endl;reset = true;}   //fps
}


int count=0;
const int nested_regions_ = 14;
int zone_[nested_regions_] = {2,3,3,3,3,3,3,2,3,3,3,3,3,3}; // Regions to extract Clusters. For more details see masterthesis
void ClusterNode::extractCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pc) {
  features_.clear();  //resets Features
  count+=1;

  // Filtering pointcloud in z-direction to remove ground and ceiling
  pcl::IndicesPtr pc_indices(new std::vector<int>);
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(pc);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_limit_min_, z_limit_max_);
  pass.filter(*pc_indices);

  // organize pointcloud to different regions
  boost::array<std::vector<int>, nested_regions_> indices_array;
  for(int i = 0; i < pc_indices->size(); i++) {
    float range = 0.0;
    for(int j = 0; j < nested_regions_; j++) {
      float d2 = pc->points[(*pc_indices)[i]].x * pc->points[(*pc_indices)[i]].x +
        pc->points[(*pc_indices)[i]].y * pc->points[(*pc_indices)[i]].y ;//+ pc->points[(*pc_indices)[i]].z * pc->points[(*pc_indices)[i]].z;
      if(d2 > range*range && d2 <= (range+zone_[j])*(range+zone_[j])) {
      	indices_array[j].push_back((*pc_indices)[i]);
        if(j<14 && ((range+zone_[j])*(range+zone_[j]))<(d2+(0.5)))
        {
            indices_array[j+1].push_back((*pc_indices)[i]);

        }
        if(j>0 && (range*range)>(d2-(0.5)) )
        {
            indices_array[j-1].push_back((*pc_indices)[i]);
        }
      	break;
      }
      range += zone_[j];
    }
  }

  float tolerance = 0.0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pre_cluster(new pcl::PointCloud<pcl::PointXYZI>);
  for(int i = 0; i < nested_regions_; i++) {
    tolerance += 0.1;  // adjust tolerance for clustering (because of vertical distance)

    // checking min cluster size
    if(indices_array[i].size() > cluster_size_min_) {

      // Clusteralgorithmus with euclidean distance (distance==tolerance)
      boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
      tree->setInputCloud(pc, indices_array_ptr);

      std::vector<pcl::PointIndices> cluster_indices;  // Vector for every cluster
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(tolerance);
      ec.setMinClusterSize(cluster_size_min_);
      ec.setMaxClusterSize(cluster_size_max_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(pc);
      ec.setIndices(indices_array_ptr);
      ec.extract(cluster_indices);


      for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);  // Defining cluster

        // writing every cluster to pointcloud
      	for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {

      	  cluster->points.push_back(pc->points[*pit]);
        }
      	cluster->width = cluster->size();
      	cluster->height = 1;
      	cluster->is_dense = true;

        // Calculating Features of cluster
        Eigen::Vector4f min, max, centroid;
        pcl::getMinMax3D(*cluster, min, max);
        pcl::compute3DCentroid(*cluster, centroid);

        Feature f;
        extractFeature(cluster, f, min, max, centroid);

        // Init visualisation of extracted cluster
         visualization_msgs::Marker marker2;
         marker2.header.frame_id = "velodyne";
         marker2.header.stamp = ros::Time();
         marker2.ns = "velodyne";
         marker2.id = 0;
         marker2.type = visualization_msgs::Marker::CUBE;
         //marker2.action = visualization_msgs::Marker::ADD;

          marker2.pose.position.z =0.0;
          marker2.pose.orientation.x = 0.0;
          marker2.pose.orientation.y = 0.0;
          marker2.pose.orientation.z = 0.0;
          marker2.pose.orientation.w = 1.0;
          marker2.scale.x = 1;
          marker2.scale.y = 0.1;
          marker2.scale.z = 0.1;
          marker2.color.a = 1.0; // Don't forget to set the alpha!
          marker2.color.r = 0.0;
          marker2.color.g = 0.0;
          marker2.color.b = 1.0;

          visualization_msgs::MarkerArray marker_array;

        // transforms polygon to right frame
        int test=0 ;
        tf::TransformListener listener;
        if(int n =polygon_points1.polygon.points.size()>0 )
        {
          std::vector<double> x;
          std::vector<double> y;

          for(auto points :polygon_points1.polygon.points)
          {

            tf::StampedTransform transform;
            geometry_msgs::PointStamped point_base_link;
            geometry_msgs::PointStamped point_velodyne;
            point_base_link.header=polygon_points1.header;
            point_base_link.header.frame_id="blu";
            point_base_link.point.x=points.x;
            point_base_link.point.y=points.y;
            point_base_link.point.z=points.z;


             try{

              _listener.transformPoint("bla",point_base_link,point_velodyne);

              x.push_back(point_velodyne.point.x);
              y.push_back(point_velodyne.point.y);
              marker2.pose.position.x = point_velodyne.point.x;
              marker2.pose.position.y = point_velodyne.point.y;

              marker_array.markers.push_back(marker2);
              marker2.id++;


              //std::cout<<"point_velodyne.point.x: "<<point_velodyne.point.x<<" point_velodyne.point.y: "<<point_velodyne.point.y<<std::endl;
             }
             catch (tf::TransformException ex){
               ROS_ERROR("%s",ex.what());
               ros::Duration(1.0).sleep();
                 break;
             }

          }
          vis_array_pub.publish(marker_array);

          x.push_back(x.at(0));
          y.push_back(y.at(0));

         test =InPolygon(x.size(),x,y,centroid[0],centroid[1]);

        }
        else{
          std::cout<<"no transform or polygon";
        }


       double min_dist=sqrt(f.min_distance);   //debugging

       // checking min. size of human
       if(test  && cluster->size()>5         &&   max[0]-min[0] >= vfilter_min_x_ && max[0]-min[0] <= vfilter_max_x_ &&
          max[1]-min[1] >= vfilter_min_y_ && max[1]-min[1] <= vfilter_max_y_ &&
            max[2]-min[2] >= vfilter_min_z_ && max[2]-min[2] <= vfilter_max_z_ && min[2] <= cluster_min_z_
                                         )
        {


           // can be added to filter
//         && (min_dist< 1.9 || min_dist > 2.1)&& (min_dist< 4.9 || min_dist > 5.1)
//          && (min_dist< 7.9 || min_dist > 8.1)&& (min_dist< 10.9 || min_dist > 11.1)


         // visualisation of cluster
         visualization_msgs::Marker marker;
         marker.header.frame_id = "velodyne";
         marker.header.stamp = ros::Time();
         marker.ns = "velodyne";
         marker.id = 0;
         marker.type = visualization_msgs::Marker::SPHERE;
         marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.z =0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
         marker.pose.position.x = centroid[0];
         marker.pose.position.y = centroid[1];
         vis_pub.publish( marker );


         // saving cluster to filter and sorting it by distance
         std::string save_dir="/home/alex/human_sort/dist_<";
         std::string save_fol;
         std::string save_trenn="/";
         if(sqrt(f.min_distance)<9)
         {
           save_fol=std::to_string((int)ceil(sqrt(f.min_distance)));

         }
         else
         {
           save_fol=std::to_string(10);
         }

           // counting files in directory
            DIR *dp;
            int k = 0;
            struct dirent *ep;
            dp = opendir ((save_dir+save_fol).c_str());

            if (dp != NULL)
            {
              while (ep = readdir (dp))
                k++;

              (void) closedir (dp);
            }
            else
              perror ("Couldn't open the directory");

            //printf("There's %d files in the current directory.\n", k);
            std::string save_path ="/home/alex/human1/";
            std::string save_name ="human";
            std::string save_no=std::to_string(k-1);
            std::string save_end=".pcd";
            try {
                 pcl::io::savePCDFileASCII(save_dir+save_fol+save_trenn+save_name+save_no+save_end,*cluster);
                  std::cout<<"x: "<<centroid[0]<<" y: "<<centroid[1]<<" test: "<<test<<" f.min_distance: "<<sqrt(f.min_distance)<<" cluster->size(): "<<cluster->size()<<"  "<<" x: "<<max[0]-min[0]<<" y: "<<max[1]-min[1]<<" z: "<<max[2]-min[2]<<std::endl;
            } catch (ros::Exception e) {

            }


            // std::cerr << "Saved " << cluster->points.size() << " data points to " << save_name+save_no+save_end <<" from "<<i <<" and Size "<< indices_array[i].size()<<" cloud: "<<count<<std::endl;

              // Debugging to view cluster
//             pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//             viewer.showCloud (cluster);

//              pcl::PointCloud<pcl::PointXYZ>::Ptr p_cluster(new pcl::PointCloud<pcl::PointXYZ>);
//              pcl::copyPointCloud(*cluster,*p_cluster);

//             p_viewer.addPointCloud(p_cluster, "cloud");

//             p_viewer.setCameraPosition(10-centroid[0],0,0,0,3,0,0);


//            p_viewer.spinOnce(1);
//             while(!p_viewer.wasStopped())
//             {
//                p_viewer.spinOnce(1);

//             }
//             std::cerr<<"close viewer"<<std::endl;
//             p_viewer.removePointCloud("cloud");


        }
        else
        {


        }
      }
    }
  }
}

/* *** Feature Extraction ***
 * f1 (1d): the number of points included in a cluster.
 * f2 (1d): the minimum distance of the cluster to the sensor.
 * => f1 and f2 should be used in pairs, since f1 varies with f2 changes.
 * f3 (6d): 3D covariance matrix of the cluster.
 * f4 (6d): the normalized moment of inertia tensor.
 * => Since both f3 and f4 are symmetric, we only use 6 elements from each as features.
 * f5 (9d): 2D covariance matrix in 3 zones, which are the upper half, and the left and right lower halves.
 * f6 (98d): The normalized 2D histogram for the main plane, 14 × 7 bins.
 * f7 (45d): The normalized 2D histogram for the secondary plane, 9 × 5 bins.
 * f8 (20d): Slice feature for the cluster.
 * f9 (27d): Intensity.
 */

void computeMomentOfInertiaTensorNormalized(pcl::PointCloud<pcl::PointXYZI> &pc, Eigen::Matrix3f &moment_3d) {
  moment_3d.setZero();
  for(size_t i = 0; i < pc.size(); i++) {
    moment_3d(0,0) += pc[i].y*pc[i].y+pc[i].z*pc[i].z;
    moment_3d(0,1) -= pc[i].x*pc[i].y;
    moment_3d(0,2) -= pc[i].x*pc[i].z;
    moment_3d(1,1) += pc[i].x*pc[i].x+pc[i].z*pc[i].z;
    moment_3d(1,2) -= pc[i].y*pc[i].z;
    moment_3d(2,2) += pc[i].x*pc[i].x+pc[i].y*pc[i].y;
  }
  moment_3d(1, 0) = moment_3d(0, 1);
  moment_3d(2, 0) = moment_3d(0, 2);
  moment_3d(2, 1) = moment_3d(1, 2);
}

/* Main plane is formed from the maximum and middle eigenvectors.
 * Secondary plane is formed from the middle and minimum eigenvectors.
 */
void computeProjectedPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Eigen::Matrix3f &eigenvectors, int axe, Eigen::Vector4f &centroid, pcl::PointCloud<pcl::PointXYZI>::Ptr plane) {
  Eigen::Vector4f coefficients;
  coefficients[0] = eigenvectors(0,axe);
  coefficients[1] = eigenvectors(1,axe);
  coefficients[2] = eigenvectors(2,axe);
  coefficients[3] = 0;
  coefficients[3] = -1 * coefficients.dot(centroid);
  for(size_t i = 0; i < pc->size(); i++) {
    float distance_to_plane =
      coefficients[0] * pc->points[i].x +
      coefficients[1] * pc->points[i].y +
      coefficients[2] * pc->points[i].z +
      coefficients[3];
    pcl::PointXYZI p;
    p.x = pc->points[i].x - distance_to_plane * coefficients[0];
    p.y = pc->points[i].y - distance_to_plane * coefficients[1];
    p.z = pc->points[i].z - distance_to_plane * coefficients[2];
    plane->points.push_back(p);
  }
}

/* Upper half, and the left and right lower halves of a pedestrian. */
void compute3ZoneCovarianceMatrix(pcl::PointCloud<pcl::PointXYZI>::Ptr plane, Eigen::Vector4f &mean, float *partial_covariance_2d) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr zone_decomposed[3];
  for(int i = 0; i < 3; i++)
    zone_decomposed[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
  for(size_t i = 0; i < plane->size(); i++) {
    if(plane->points[i].z >= mean(2)) { // upper half
      zone_decomposed[0]->points.push_back(plane->points[i]);
    } else {
      if(plane->points[i].y >= mean(1)) // left lower half
	zone_decomposed[1]->points.push_back(plane->points[i]);
      else // right lower half
	zone_decomposed[2]->points.push_back(plane->points[i]);
    }
  }

  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  for(int i = 0; i < 3; i++) {
    pcl::compute3DCentroid(*zone_decomposed[i], centroid);
    pcl::computeCovarianceMatrix(*zone_decomposed[i], centroid, covariance);
    partial_covariance_2d[i*3+0] = covariance(0,0);
    partial_covariance_2d[i*3+1] = covariance(0,1);
    partial_covariance_2d[i*3+2] = covariance(1,1);
  }
}

void computeHistogramNormalized(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int horiz_bins, int verti_bins, float *histogram) {
  Eigen::Vector4f min, max, min_box, max_box;
  pcl::getMinMax3D(*pc, min, max);
  float horiz_itv, verti_itv;
  horiz_itv = (max[0]-min[0]>max[1]-min[1]) ? (max[0]-min[0])/horiz_bins : (max[1]-min[1])/horiz_bins;
  verti_itv = (max[2] - min[2])/verti_bins;

  for(int i = 0; i < horiz_bins; i++) {
    for(int j = 0; j < verti_bins; j++) {
      if(max[0]-min[0] > max[1]-min[1]) {
	min_box << min[0]+horiz_itv*i, min[1], min[2]+verti_itv*j, 0;
	max_box << min[0]+horiz_itv*(i+1), max[1], min[2]+verti_itv*(j+1), 0;
      } else {
	min_box << min[0], min[1]+horiz_itv*i, min[2]+verti_itv*j, 0;
	max_box << max[0], min[1]+horiz_itv*(i+1), min[2]+verti_itv*(j+1), 0;
      }
      std::vector<int> indices;
      pcl::getPointsInBox(*pc, min_box, max_box, indices);
      histogram[i*verti_bins+j] = (float)indices.size() / (float)pc->size();
    }
  }
}

void computeSlice(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int n, float *slice) {
  Eigen::Vector4f pc_min, pc_max;
  pcl::getMinMax3D(*pc, pc_min, pc_max);

  pcl::PointCloud<pcl::PointXYZI>::Ptr blocks[n];
  float itv = (pc_max[2] - pc_min[2]) / n;
  if(itv > 0) {
    for(int i = 0; i < n; i++) {
      blocks[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
    }
    for(unsigned int i = 0, j; i < pc->size(); i++) {
      j = std::min((n-1), (int)((pc->points[i].z - pc_min[2]) / itv));
      blocks[j]->points.push_back(pc->points[i]);
    }

    Eigen::Vector4f block_min, block_max;
    for(int i = 0; i < n; i++) {
      if(blocks[i]->size() > 0) {
	// pcl::PCA<pcl::PointXYZI> pca;
	// pcl::PointCloud<pcl::PointXYZI>::Ptr block_projected(new pcl::PointCloud<pcl::PointXYZI>);
	// pca.setInputCloud(blocks[i]);
	// pca.project(*blocks[i], *block_projected);
	pcl::getMinMax3D(*blocks[i], block_min, block_max);
      } else {
	block_min.setZero();
	block_max.setZero();
      }
      slice[i*2] = block_max[0] - block_min[0];
      slice[i*2+1] = block_max[1] - block_min[1];
    }
  } else {
    for(int i = 0; i < 20; i++)
      slice[i] = 0;
  }
}

void computeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int bins, float *intensity) {
  float sum = 0, mean = 0, sum_dev = 0;
  float min = FLT_MAX, max = -FLT_MAX;
  for(int i = 0; i < 27; i++)
    intensity[i] = 0;

  for(size_t i = 0; i < pc->size(); i++) {
    sum += pc->points[i].intensity;
    min = std::min(min, pc->points[i].intensity);
    max = std::max(max, pc->points[i].intensity);
  }
  mean = sum / pc->size();

  for(size_t i = 0; i < pc->size(); i++) {
    sum_dev += (pc->points[i].intensity-mean)*(pc->points[i].intensity-mean);
    int ii = std::min(float(bins-1), std::floor((pc->points[i].intensity-min)/((max-min)/bins)));
    intensity[ii]++;
  }
  intensity[25] = sqrt(sum_dev/pc->size());
  intensity[26] = mean;
}

void ClusterNode::extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f,
				      Eigen::Vector4f &min, Eigen::Vector4f &max, Eigen::Vector4f &centroid) {
  f.centroid = centroid;
  f.min = min;
  f.max = max;


    // f1: Number of points included the cluster.
    f.number_points = pc->size();
    // f2: The minimum distance to the cluster.
    f.min_distance = FLT_MAX;
    float d2; //squared Euclidean distance
    for(int i = 0; i < pc->size(); i++) {
      d2 = pc->points[i].x*pc->points[i].x + pc->points[i].y*pc->points[i].y + pc->points[i].z*pc->points[i].z;
      if(f.min_distance > d2)
	f.min_distance = d2;
    }
    //f.min_distance = sqrt(f.min_distance);

    pcl::PCA<pcl::PointXYZI> pca;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_projected(new pcl::PointCloud<pcl::PointXYZI>);
    pca.setInputCloud(pc);
    pca.project(*pc, *pc_projected);
    // f3: 3D covariance matrix of the cluster.
    pcl::computeCovarianceMatrixNormalized(*pc_projected, centroid, f.covariance_3d);
    // f4: The normalized moment of inertia tensor.
    computeMomentOfInertiaTensorNormalized(*pc_projected, f.moment_3d);
    // Navarro et al. assume that a pedestrian is in an upright position.
    //pcl::PointCloud<pcl::PointXYZI>::Ptr main_plane(new pcl::PointCloud<pcl::PointXYZI>), secondary_plane(new pcl::PointCloud<pcl::PointXYZI>);
    //computeProjectedPlane(pc, pca.getEigenVectors(), 2, centroid, main_plane);
    //computeProjectedPlane(pc, pca.getEigenVectors(), 1, centroid, secondary_plane);
    // f5: 2D covariance matrix in 3 zones, which are the upper half, and the left and right lower halves.
    //compute3ZoneCovarianceMatrix(main_plane, pca.getMean(), f.partial_covariance_2d);
    // f6 and f7
    //computeHistogramNormalized(main_plane, 7, 14, f.histogram_main_2d);
    //computeHistogramNormalized(secondary_plane, 5, 9, f.histogram_second_2d);
    // f8
    computeSlice(pc, 10, f.slice);
    // f9
    computeIntensity(pc, 25, f.intensity);

}




int main(int argc, char **argv) {
  ros::init(argc, argv, "cluster_node");
  ClusterNode d;
  ros::spin();
  return 0;
}
