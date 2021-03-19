/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the GPL-style license found in the
LICENSE file in the root directory of this source tree.
*/
/*!
  *\file: training_ob_multi_fast.cpp
  * \author: Alexander Petersen
  * \date: 21.11.19
  * \brief: TrainingOBMulti Classe:
  * Class to train SVM with 5 features
  */

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
// SVM
#include "svm.h"


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
  //float intensity[27];
} Feature;

static const int FEATURE_SIZE = 34;  /**< Dimension of Featurevector */
float svm_range_[FEATURE_SIZE][2];  /**< rangearrry for rescaling features*/
float svm_xlower_ = -1.0, svm_xupper_ = 1.0;  /**< lower and upper limit of range */

class TrainingObMulti {
private:
  /// Nodehandle
  ros::NodeHandle node_handle_;

  /// Parameter for filtering cluster
  float vfilter_min_x_;
  float vfilter_max_x_;
  float vfilter_min_y_;
  float vfilter_max_y_;
  float vfilter_min_z_;
  float vfilter_max_z_;
  float cluster_min_z_;

  /// SVM parameter
  std::vector<Feature> features_;  /**< global featurevector for every pointcloud */
  struct svm_model *svm_model_;    /**< SVM Model */
  struct svm_problem svm_problem_; /**< Defines SVM problem (features for every labeled class)*/
  struct svm_parameter svm_parameter_; /**< Defines SVM parameters*/
  int svm_node_size_;   /**< Node size for allocate memory*/
  bool find_the_best_training_parameters_; /**< enabels cross-validation*/

  /// Files
  std::string save_name ="human";  /**< name of pcd file*/
  std::string save_end=".pcd";  /**< end tag*/
  std::vector<std::string> path_vec {"/home/alex/training1_2/","/home/alex/training3_2/","/home/alexr/training2_2/"}; /**< vector of directories for human pcds*/
  std::vector<std::string> path_vec_no_human {"/home/alex/no_human/"};  /**< vector of directories for no human pcds*/
  std::string save_name_no_human ="no_human";  /**< name of no_human files*/
public:
  /** Creating TrainingObMulti Class
   * Initialising all parameters and starts training
 */
  TrainingObMulti();
  ~TrainingObMulti();

  /** \brief Extract features.
    * \param pc Pointcloud
    * \param f Featurevector
    *
    * This method calculate Features from given cluster
    */
  void extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f);

  /** \brief Save features.
    * \param f Featurevector
    * \param x Featurevector in SVM format
    *
    * This method is converting the calculated features to a SVM format
    */
  void saveFeature(Feature &f, struct svm_node *x);

  /** \brief train.
    *
    * This method trains a SVM
    */
  void train();

  /** \brief Load Data.
    *
    * This method load human and no-human pcds to train a model
    */
  void load_data();
};

TrainingObMulti::TrainingObMulti() {
  /*** Nodehandle ***/
  ros::NodeHandle private_nh("~");

  /*** Parameters ***/
  private_nh.param<bool>("find_the_best_training_parameters", find_the_best_training_parameters_, true);

  private_nh.param<float>("vfilter_min_x", vfilter_min_x_, 0.2);
  private_nh.param<float>("vfilter_max_x", vfilter_max_x_, 1.0);
  private_nh.param<float>("vfilter_min_y", vfilter_min_y_, 0.2);
  private_nh.param<float>("vfilter_max_y", vfilter_max_y_, 1.0);
  private_nh.param<float>("vfilter_min_z", vfilter_min_z_, 0.2);
  private_nh.param<float>("vfilter_max_z", vfilter_max_z_, 2.0);
  private_nh.param<float>("cluster_min_z", cluster_min_z_, 2.0);

  private_nh.param<std::string>("/show_pcd/save_name", save_name, "human");
  private_nh.param<std::string>("/show_pcd/save_end", save_end, ".pcd");

  /*** SVM ***/
  svm_parameter_.svm_type = C_SVC; // default C_SVC
  svm_parameter_.kernel_type = RBF; // default RBF
  svm_parameter_.degree = 3; // default 3
  svm_parameter_.gamma = 0.02; // default 1.0/(float)FEATURE_SIZE
  svm_parameter_.coef0 = 0; // default 0
  svm_parameter_.cache_size = 256; // default 100
  svm_parameter_.eps = 0.001; // default 0.001
  svm_parameter_.C = 8; // default 1
  svm_parameter_.nr_weight = 0;
  svm_parameter_.weight_label = NULL;
  svm_parameter_.weight = NULL;
  svm_parameter_.nu = 0.5;
  svm_parameter_.p = 0.1;
  svm_parameter_.shrinking = 0;
  svm_parameter_.probability = 1;

  // allocate memory
  svm_node_size_ = 8000;
  svm_problem_.l = 0;
  svm_problem_.y = (double *)malloc(svm_node_size_*sizeof(double));
  std::cerr<<"prob y size: "<<svm_node_size_*sizeof(double)<<std::endl;
  svm_problem_.x = (struct svm_node **)malloc(svm_node_size_*sizeof(struct svm_node *));
  std::cerr<<"prob x size: "<<svm_node_size_*sizeof(struct svm_node *)<<std::endl;
  for(int i = 0; i < svm_node_size_; i++) {
    svm_problem_.x[i] = (struct svm_node *)malloc((FEATURE_SIZE + 1)*sizeof(struct svm_node));
  }


  load_data();
}

TrainingObMulti::~TrainingObMulti() {
  // free memory
  svm_free_and_destroy_model(&svm_model_);
  svm_destroy_param(&svm_parameter_);
  free(svm_problem_.y);
  for(int i = 0; i < svm_node_size_; i++)
    free(svm_problem_.x[i]);
  free(svm_problem_.x);
}


void TrainingObMulti::load_data()
{
    features_.clear();  // clearing all features


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    //*********** load Human pointclouds ***********

    // iterates through all directories
    for (int b=0;b<path_vec.size();b++) {

          //counting all files in directory
          DIR *dp;
          int i = 0;
          struct dirent *ep;


          dp = opendir (path_vec[b].c_str());

          if (dp != NULL)
          {
            while (ep = readdir (dp))
              i++;

            (void) closedir (dp);
          }
          else
            perror ("Couldn't open the directory");
            perror(path_vec[b].c_str());
          //printf("There's %d files in the current directory.\n", i);


          // iterates through all files
          for (int u=1;u<i-1;u++)
          {
              // loading file
              std::string save_no=std::to_string(u);
              if (pcl::io::loadPCDFile<pcl::PointXYZI> (path_vec[b]+save_name+save_no+save_end, *cloud) == -1) //* load the file
              {
                  continue;
              }
              std::cerr<<path_vec[b]+save_name+save_no+save_end<<std::endl;
              cloud->width = cloud->size();
              cloud->height = 1;
              cloud->is_dense = true;

              // calculate features
              Eigen::Vector4f min, max;
              pcl::getMinMax3D(*cloud, min, max);

              // filtering
              if(max[0]-min[0] >= vfilter_min_x_ && max[0]-min[0] <= vfilter_max_x_ &&
                 max[1]-min[1] >= vfilter_min_y_ && max[1]-min[1] <= vfilter_max_y_ &&
                 max[2]-min[2] >= vfilter_min_z_ && max[2]-min[2] <= vfilter_max_z_ &&
                 min[2] <= cluster_min_z_) {
                      Feature f;
                      extractFeature(cloud, f);
                      features_.push_back(f);
                      //std::cerr<<" svm_problem l: "<<svm_problem_.l<<" svm_problem y: "<<*svm_problem_.y<<std::endl;
                      saveFeature(f, svm_problem_.x[svm_problem_.l]);
                      std::string bla="+1";
                      svm_problem_.y[svm_problem_.l++] = 1;
                      std::cerr<<" svm_problem l: "<<svm_problem_.l<<" svm_problem y: "<<*svm_problem_.y<<std::endl;
              } else {
                      std::cerr<<"filtering aktiv"<<std::endl;
                      std::cerr<<path_vec[b]+save_name+save_no+save_end<<std::endl;

              }
               std::cerr<<path_vec[b]+save_name+save_no+save_end<<std::endl;
              cloud->clear();
          }
    }

    /*** save data to file ***/
     std::ofstream s;
     s.open("svm_training_data_test_multi_fast");
     for(int i = 0; i < svm_problem_.l; i++) {
       s << "+1";
       for(int j = 0; j < FEATURE_SIZE; j++)
         s << " " << svm_problem_.x[i][j].index << ":" <<  svm_problem_.x[i][j].value;
       s << "\n";
     }
     s.close();
    std::cerr<<"saved svm_problem"<<std::endl;
    int saved_pos=svm_problem_.l;

    //********************************************** No-Human *******************************************
    //load every directory
    for (int b=0;b<path_vec_no_human.size();b++) {

        //counting files
        DIR *dp;
        int i = 0;
        struct dirent *ep;


        dp = opendir (path_vec_no_human[b].c_str());

        if (dp != NULL)
        {
          while (ep = readdir (dp))
            i++;

          (void) closedir (dp);
        }
        else
          perror ("Couldn't open the directory");

        //printf("There's %d files in the current directory.\n", i);

         //iterates through every file
        for (int u=1;u<i-1;u++)
        {
            // loading file
            std::string save_no=std::to_string(u);
            if (pcl::io::loadPCDFile<pcl::PointXYZI> (path_vec_no_human[b]+save_name_no_human+save_no+save_end, *cloud) == -1) //* load the file
            {

                continue;
            }
            std::cerr<<path_vec_no_human[b]+save_name_no_human+save_no+save_end<<std::endl;
            cloud->width = cloud->size();
            cloud->height = 1;
            cloud->is_dense = true;

            //calculate features and filtering
            Eigen::Vector4f min, max;
            pcl::getMinMax3D(*cloud, min, max);
            if(max[0]-min[0] >= vfilter_min_x_ && max[0]-min[0] <= vfilter_max_x_ &&
               max[1]-min[1] >= vfilter_min_y_ && max[1]-min[1] <= vfilter_max_y_ &&
               max[2]-min[2] >= vfilter_min_z_ && max[2]-min[2] <= vfilter_max_z_ &&
               min[2] <= cluster_min_z_) {
              Feature f;
              extractFeature(cloud, f);
              features_.push_back(f);
              //std::cerr<<" svm_problem l: "<<svm_problem_.l<<" svm_problem y: "<<*svm_problem_.y<<std::endl;
              saveFeature(f, svm_problem_.x[svm_problem_.l]);
              std::string bla="-1";
              svm_problem_.y[svm_problem_.l++] = -1;
              std::cerr<<" svm_problem l: "<<svm_problem_.l<<" svm_problem y: "<<*svm_problem_.y<<std::endl;
            } else {
                std::cerr<<"filtering aktiv"<<std::endl;
                    std::cerr<<path_vec_no_human[b]+save_name_no_human+save_no+save_end<<std::endl;

            }
             std::cerr<<path_vec_no_human[b]+save_name_no_human+save_no+save_end<<std::endl;
             cloud->clear();
        }
    }


    /*** save data to file ***/
    std::fstream fs;
    fs.open ("svm_training_data_test_multi_fast", std::fstream::in | std::fstream::out | std::fstream::app);
     for(int i = saved_pos; i < svm_problem_.l; i++) {
       fs << "-1";
       for(int j = 0; j < FEATURE_SIZE; j++)
         fs << " " << svm_problem_.x[i][j].index << ":" <<  svm_problem_.x[i][j].value;
       fs << "\n";
     }
       fs.close();
    std::cerr<<"saved svm_problem"<<std::endl;
     train();
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

void TrainingObMulti::extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f) {
  Eigen::Vector4f centroid, min, max;
  pcl::compute3DCentroid(*pc, centroid);
  pcl::getMinMax3D(*pc, min, max);

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
  // // Navarro et al. assume that a pedestrian is in an upright position.
  // pcl::PointCloud<pcl::PointXYZI>::Ptr main_plane(new pcl::PointCloud<pcl::PointXYZI>), secondary_plane(new pcl::PointCloud<pcl::PointXYZI>);
  // computeProjectedPlane(pc, pca.getEigenVectors(), 2, centroid, main_plane);
  // computeProjectedPlane(pc, pca.getEigenVectors(), 1, centroid, secondary_plane);
  // // f5: 2D covariance matrix in 3 zones, which are the upper half, and the left and right lower halves.
  // compute3ZoneCovarianceMatrix(main_plane, pca.getMean(), f.partial_covariance_2d);
  // // f6 and f7
  // computeHistogramNormalized(main_plane, 7, 14, f.histogram_main_2d);
  // computeHistogramNormalized(secondary_plane, 5, 9, f.histogram_second_2d);
  // f8
  computeSlice(pc, 10, f.slice);
  // f9
  //computeIntensity(pc, 25, f.intensity);
}

void TrainingObMulti::saveFeature(Feature &f, struct svm_node *x) {
  x[0].index  = 1;  x[0].value  = f.number_points; // libsvm indices start at 1
  x[1].index  = 2;  x[1].value  = f.min_distance;
  x[2].index  = 3;  x[2].value  = f.covariance_3d(0,0);
  x[3].index  = 4;  x[3].value  = f.covariance_3d(0,1);
  x[4].index  = 5;  x[4].value  = f.covariance_3d(0,2);
  x[5].index  = 6;  x[5].value  = f.covariance_3d(1,1);
  x[6].index  = 7;  x[6].value  = f.covariance_3d(1,2);
  x[7].index  = 8;  x[7].value  = f.covariance_3d(2,2);
  x[8].index  = 9;  x[8].value  = f.moment_3d(0,0);
  x[9].index  = 10; x[9].value  = f.moment_3d(0,1);
  x[10].index = 11; x[10].value = f.moment_3d(0,2);
  x[11].index = 12; x[11].value = f.moment_3d(1,1);
  x[12].index = 13; x[12].value = f.moment_3d(1,2);
  x[13].index = 14; x[13].value = f.moment_3d(2,2);
  // for(int i = 0; i < 9; i++) {
  //   x[i+14].index = i+15;
  //   x[i+14].value = f.partial_covariance_2d[i];
  // }
  // for(int i = 0; i < 98; i++) {
  //   x[i+23].index = i+24;
  //   x[i+23].value = f.histogram_main_2d[i];
  // }
  // for(int i = 0; i < 45; i++) {
  //   x[i+121].index = i+122;
  //   x[i+121].value = f.histogram_second_2d[i];
  // }
  for(int i = 0; i < 20; i++) {
    x[i+14].index = i+15;
    x[i+14].value = f.slice[i];
  }
//  for(int i = 0; i < 27; i++) {
//    x[i+34].index = i+35;
//    x[i+34].value = f.intensity[i];
//  }
  x[FEATURE_SIZE].index = -1;

  // for(int i = 0; i < FEATURE_SIZE; i++) {
  //   std::cerr << x[i].index << ":" << x[i].value << " ";
  //   std::cerr << std::endl;
  // }
}



void TrainingObMulti::train() {

 std::cerr << "Training round " << "started" << std::endl;
  clock_t t = clock();

  /*** scale the current data ***/
  for(int i = 0; i < FEATURE_SIZE; i++) {
    svm_range_[i][0] = FLT_MAX; // min range
    svm_range_[i][1] = -FLT_MAX; // max range
  }
  for(int i = 0; i < svm_problem_.l; i++) {
    for(int j = 0; j < FEATURE_SIZE; j++) {
      svm_range_[j][0] = std::min(svm_range_[j][0], (float)svm_problem_.x[i][j].value);
      svm_range_[j][1] = std::max(svm_range_[j][1], (float)svm_problem_.x[i][j].value);
    }
    /*** save data to file ***/
     std::ofstream s;
     s.open("range_test_multi_new_fast");
     //s.open(boost::to_string(train_round_).c_str());
     s<<"x\n";
     s<<"-1 1\n";
     for(int i = 0; i <FEATURE_SIZE; i++) {
       s << i+1<<" ";
        s<<svm_range_[i][0]<<" ";
        s<<svm_range_[i][1];
       s << "\n";
     }
     s.close();
  }

  /*** debug print ***/
//   for(int i = 0; i < FEATURE_SIZE; i++)
//     std::cerr << "svm scale range [attribute " << i << "]: " << svm_range_[i][0] << ", " << svm_range_[i][1] << std::endl;

  // writing svm_problem
  for(int i = 0; i < svm_problem_.l; i++) {
    for(int j = 0; j < FEATURE_SIZE; j++) {
      if(svm_range_[j][0] == svm_range_[j][1]) // skip single-valued attribute
        continue;
      if(svm_problem_.x[i][j].value == svm_range_[j][0])
        svm_problem_.x[i][j].value = svm_xlower_;
      else if(svm_problem_.x[i][j].value == svm_range_[j][1])
        svm_problem_.x[i][j].value = svm_xupper_;
      else
        svm_problem_.x[i][j].value = svm_xlower_ + (svm_xupper_ - svm_xlower_) * (svm_problem_.x[i][j].value - svm_range_[j][0]) / (svm_range_[j][1] - svm_range_[j][0]);
     // std::cerr << "training data " << i << " [attribute " << j << "]: " << svm_problem_.x[i][j].value << std::endl;
    }
  }




  /*** train ***/

   /*** save data to file ***/
  if(find_the_best_training_parameters_) {
    std::ofstream s;
    s.open("svm_training_data_new_fast");
    for(int i = 0; i < svm_problem_.l; i++) {
      s << svm_problem_.y[i];
      for(int j = 0; j < FEATURE_SIZE; j++)
        s << " " << svm_problem_.x[i][j].index << ":" <<  svm_problem_.x[i][j].value;
      s << "\n";
    }
    s.close();

    /*** cross-validation ***/
    std::cerr << "Finding the best training parameters ..." << std::endl;
    //std::cerr <<svm_check_parameter(&svm_problem_, &svm_parameter_) <<std::endl;
    if(svm_check_parameter(&svm_problem_, &svm_parameter_) == NULL) {
      char result[100];
      std::cerr << "open grid.py" << std::endl;
      FILE *fp = popen("./grid.py svm_training_data_new", "r");
      if(fp == NULL) {
        std::cerr << "Can not run cross validation!" << std::endl;
        std::cerr << "test1" << std::endl;
      } else {
          std::cerr << "test2" << std::endl;
        if(fgets(result, 100, fp) != NULL) {
          char *pch = strtok(result, " ");
          svm_parameter_.C = atof(pch); pch = strtok(NULL, " ");
          svm_parameter_.gamma = atof(pch); pch = strtok(NULL, " ");
          float rate = atof(pch);
          std::cerr << "Best c=" << svm_parameter_.C << ", g=" << svm_parameter_.gamma << " CV rate=" << rate << std::endl;
        }
      }
      pclose(fp);
    }
  }

  //train
  svm_model_ = svm_train(&svm_problem_, &svm_parameter_);

  std::cerr << "Training round "  << " finished with " << float(clock()-t)/CLOCKS_PER_SEC << " seconds" << std::endl;

   std::cerr << "svm_get_svm_type " << svm_get_svm_type(svm_model_) <<  std::endl;
   std::cerr << "svm_get_nr_class " << svm_get_nr_class(svm_model_) <<  std::endl;
   std::cerr << "svm_get_nr_sv " << svm_get_nr_sv(svm_model_) <<  std::endl;

  std::cerr << "svm_get_svr_probability " << svm_get_svr_probability(svm_model_) <<  std::endl;
  std::cerr << "svm_check_probability_model " << svm_check_probability_model(svm_model_) <<  std::endl;




  /*** debug saving ***/
if(svm_save_model("pedestrian_test_multi_new_fast.model", svm_model_) == 0)
   std::cerr << "A model has been generated here: ~/.ros/pedestrian_test_multi_new_fast.model" << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traning_ob_multi_fast");
  TrainingObMulti d;
  /*** single-thread ***/
  ros::spin();
  /*** multi-thread ***/
  //ros::MultiThreadedSpinner spinner(4);
  //spinner.spin();
  return 0;
}
