/* *****************************************************************
 *
 * vis_path
 *
 * Copyright (c) %YEAR%,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 ******************************************************************/

/*!
  *\file: detection_com.cpp
  * \author: Alexander Petersen
  * \date: 21.11.19
  * \brief: DetectionCom Class:
  * Class to fuse all detections
  */

#ifndef DETECTION_COM_DETECTION_COM_H
#define DETECTION_COM_DETECTION_COM_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/GetPlan.h>
#include <tf/transform_listener.h>
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <cmath>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
#include <sg_msgs/DetectedPersonsOdom.h>

class DetectionCom
{
public:
    DetectionCom(ros::NodeHandle &node_handle);

private:
    // node handle
    ros::NodeHandle *node_;

    // ros communication
    ros::Subscriber my_subscriber_;
    ros::Publisher my_publisher_;
    ros::ServiceServer my_service_server_;
    ros::ServiceClient my_service_client_;
    ros::Timer my_timer_;
    ros::Publisher marker_pub;

    ros::Subscriber laser_3d_sub;
    ros::Subscriber laser_3d_fast_sub;
    ros::Subscriber cam_front_sub;
    ros::Subscriber cam_back_sub;
    ros::Subscriber laser_2d_sub;

    ros::Publisher pub_3D;
    ros::Publisher pub_cam_front;
    ros::Publisher pub_cam_back;
    ros::Publisher pub_2D;

    ros::Publisher spencer_pub;

    spencer_tracking_msgs::DetectedPersons global_detections;

    sg_msgs::DetectedPersonsOdom global_detections_odom_2D;
    sg_msgs::DetectedPersonsOdom global_detections_odom_3D;
    sg_msgs::DetectedPersonsOdom global_detections_odom_3D_fast;
    sg_msgs::DetectedPersonsOdom global_detections_odom_cam_front;
    sg_msgs::DetectedPersonsOdom global_detections_odom_cam_back;

    tf::TransformListener listener;


    boost::mutex mutex;
    Eigen::Vector3d trans;
     Eigen::Matrix3d rot;

     Eigen::Vector3d trans_cam_front;
      Eigen::Matrix3d rot_cam_front;

      Eigen::Vector3d trans_cam_back;
       Eigen::Matrix3d rot_cam_back;

       Eigen::Vector3d trans_laser_2d;
        Eigen::Matrix3d rot_laser2d;

     bool velo_pushed=false;
     bool velo_fast_pushed=false;
     bool cam_front_pushed=false;
     bool cam_back_pushed=false;
     bool laser2d_pushed=false;

     bool got_trans_velo=false;
     bool got_trans_cam_front=false;
     bool got_trans_cam_back=false;
     bool got_trans_2dlaser=false;

    // parameters
    double vis_time;
    std::string my_string_parameter_;
    int my_int_parameter_;
    bool my_bool_parameter_;

    // callbacks
    void subscriberCallback(const geometry_msgs::Point &msg);
    //bool serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void timerCallback(const ros::TimerEvent &evt);

    void laser_3d_cb(const sg_msgs::DetectedPersonsOdomConstPtr &msg);
    void laser_3d_fast_cb(const sg_msgs::DetectedPersonsOdomConstPtr &msg);
    void cam_front_cb(const sg_msgs::DetectedPersonsOdomConstPtr &msg);
    void cam_back_cb(const sg_msgs::DetectedPersonsOdomConstPtr &msg);
    void laser_2d_cb(const sg_msgs::DetectedPersonsOdomConstPtr &msg);
};

#endif // DETECTION_COM_DETECTION_COM_H
