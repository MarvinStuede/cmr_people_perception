/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the GPL-style license found in the
LICENSE file in the root directory of this source tree.
*/
/*!
  *\file: show_pcd.cpp
  * \author: Alexander Petersen
  * \date: 21.11.19
  * \brief: Object3dDetector Classe:
  * Class to show cluster file and delete them
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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
// SVM


#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <iostream>
#include <fstream>
#include <cstdio>
//#include <curses.h>
#include <sys/select.h>
#include <termios.h>
#include <stropts.h>
#include <sys/ioctl.h>


class ShowPcd {
private:

  ros::NodeHandle node_handle_;

  /// pcl viewer
  pcl::visualization::PCLVisualizer p_viewer;

  /// parameters to read files
  std::string save_path ="/home/alex/human_sort/dist<5/"; /**< path */
  std::string save_name ="human";
  std::string save_end=".pcd";
  bool delete_cloud;
  double wait_time;

  
public:
  /** Creating ShowPcd Class
   * Initialising all parameters and shows pointcloud clusters
 */
  ShowPcd();
  ~ShowPcd();



  /** \brief rename
    * \return Integer of how many files the directory contants
    *
    * This method renames all files in a directory from 1 to the count of files
    */
  int rename();
  
};



/** \brief getch
  * \return Char of pressed keyboard
  *
  * This method returns keyboard input from user
  */
char getch(){
    char buf=0;
    struct termios old={0};
    fflush(stdout);
    if(tcgetattr(0, &old)<0)
        perror("tcsetattr()");
    old.c_lflag&=~ICANON;
    old.c_lflag&=~ECHO;
    old.c_cc[VMIN]=1;
    old.c_cc[VTIME]=0;
    if(tcsetattr(0, TCSANOW, &old)<0)
        perror("tcsetattr ICANON");
    if(read(0,&buf,1)<0)
        perror("read()");
    old.c_lflag|=ICANON;
    old.c_lflag|=ECHO;
    if(tcsetattr(0, TCSADRAIN, &old)<0)
        perror ("tcsetattr ~ICANON");
    printf("%c\n",buf);
    return buf;
 }

ShowPcd::ShowPcd() {

  ros::NodeHandle private_nh("show_pcd");

  /*** Parameters ***/
  private_nh.param<std::string>("/show_pcd/save_path", save_path, "/home/alex/human_sort/dist_<7/");
  private_nh.param<std::string>("/show_pcd/save_name", save_name, "human");
  private_nh.param<std::string>("/show_pcd/save_end", save_end, ".pcd");
   private_nh.param<bool>("/show_pcd/delete", delete_cloud, true);
    private_nh.param<double>("/show_pcd/wait", wait_time, 0.5);

  
     // counts files in directory
      DIR *dp;
      int i = 0;
      struct dirent *ep;


      dp = opendir (save_path.c_str());

      if (dp != NULL)
      {
        while (ep = readdir (dp))
          i++;

        (void) closedir (dp);
      }
      else
        perror ("Couldn't open the directory");
        std::cout<<save_path<<std::endl;

      //printf("There's %d files in the current directory.\n", i);


      // iterates through every file in directory
      for (int u=1;u<i-1;u++)
      {

              std::string save_no=std::to_string(u);

              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

              if (pcl::io::loadPCDFile<pcl::PointXYZ> (save_path+save_name+save_no+save_end, *cloud) == -1) //* load the file
              {

                  continue;
              }

              // calculate some features of cluster (size, position)
              Eigen::Vector4f min, max, centroid;
              pcl::getMinMax3D(*cloud, min, max);
              pcl::compute3DCentroid(*cloud, centroid);
              float min_distance = FLT_MAX;
              float d2; //squared Euclidean distance
              for(int i = 0; i < cloud->size(); i++) {
                d2 = cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y + cloud->points[i].z*cloud->points[i].z;
                if(min_distance > d2)
                  min_distance = d2;
              }
              min_distance = sqrt(min_distance);

              std::cout << "Loaded "
                        << cloud->width * cloud->height
                        << " data points "<<"with min distance: "<<min_distance<< " from "<< save_name+save_no+save_end<<" with the following fields: "
                        << std::endl;

              // view cloud
              p_viewer.addPointCloud(cloud, "cloud");

              p_viewer.setCameraPosition(centroid.x()-5, centroid.y() , centroid.z(), 0, 0, 1);

              std::vector<pcl::visualization::Camera> cams;
              p_viewer.getCameras(cams);

              for (auto&& camera : cams)
              {
                  camera.focal[0] = centroid.x();
                  camera.focal[1] = centroid.y();
                  camera.focal[2] = centroid.z();
              }

              p_viewer.setCameraParameters(cams[0]);


              // get input from user (q -> go into viewer for more details, a -> previous cloud, z -> delete cloud, every other key -> next cloud
               char input='m';
               if(delete_cloud)
                {
                    input=0;
                    while(!input)
                    {
                       input = getch();
                       //view cloud
                        if(input=='q')
                        {
                            p_viewer.spin();
                            input=0;
                            continue;
                        }
                        else if (input=='a')
                        {
                            if(u>1)
                              u=u-2;
                            else if(u==1)
                            {
                                u=u-1;
                            }


                        }
                        else if (input=='z')
                        {
                            std::string path = save_path+save_name+save_no+save_end;

                            if( std::remove( path.c_str()) != 0 )
                                perror( "Error deleting file" );
                              else
                                puts( "File successfully deleted" );
                            i=rename();
                            if(u>0)
                              u=u-1;
                        }




                    }

                }


                p_viewer.removePointCloud("cloud");
                ros::Duration(wait_time).sleep();    //time to wait between next cloud



         }

      //******rename files******
        rename();


        ros::shutdown();  // close node

}

int ShowPcd::rename(){

    DIR *dp;
    int z = 0;
    struct dirent *ep;


    dp = opendir (save_path.c_str());

    if (dp != NULL)
    {
      while (ep = readdir (dp))
        z++;

      (void) closedir (dp);
    }
    else
      perror ("Couldn't open the directory");

    //printf("There's %d files in the current directory.\n", i);


    int p=1;
    for (int u=1;u<z-1;u++)
    {
        p=u;
        std::string save_no=std::to_string(u);
        std::string p_string=std::to_string(p);
        std::string bla=save_path+save_name+save_no+save_end;
        std::string blu=save_path+save_name+p_string+save_end;

            if(access( bla.c_str(), F_OK ) != -1)
            {
                  //std::cerr<<"gut: "<<save_no<<std::endl;
            }
        else {
           while(access( blu.c_str(), F_OK ) == -1 )
           {
               //std::cerr<<"schlecht: "<<p_string<<std::endl;
               p++;
               p_string=std::to_string(p);
              blu=save_path+save_name+p_string+save_end;
           }
           std::string path_to_save = save_path+save_name+save_no+save_end;
           std::string path_wrong=save_path+save_name+p_string+save_end;
           std::rename(path_wrong.c_str(),path_to_save.c_str());
        }
    }
    return z;
}

ShowPcd::~ShowPcd() {

}





int main(int argc, char **argv) {
  ros::init(argc, argv, "show_pcd");
  ShowPcd d;
  ros::spin();
  return 0;
}
