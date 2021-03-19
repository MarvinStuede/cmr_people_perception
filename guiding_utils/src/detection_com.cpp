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

#include "detection_com.h"
#include <tf_conversions/tf_eigen.h>


//########## CONSTRUCTOR ###############################################################################################
DetectionCom::DetectionCom(ros::NodeHandle &node_handle):
    node_(&node_handle)
{

    // === Publisher ===
    pub_cam_front=node_->advertise<spencer_tracking_msgs::DetectedPersons>("/spencer/perception_internal/detected_persons/rgbd_front_top/upper_body",10);
    pub_cam_back=node_->advertise<spencer_tracking_msgs::DetectedPersons>("/spencer/perception_internal/detected_persons/rgbd_rear_top/upper_body",10);
    pub_3D=node_->advertise<spencer_tracking_msgs::DetectedPersons>("/spencer/perception_internal/detected_persons/velodyne",10);
    //spencer_pub=node_->advertise<spencer_tracking_msgs::DetectedPersons>("/spencer/perception_internal/detected_persons/laser_front",10);


    // === SUBSCRIBERS ===
    laser_3d_sub = node_->subscribe("/spencer/perception_internal/detected_persons/pre/velodyne", 10, &DetectionCom::laser_3d_cb, this);
    laser_3d_fast_sub = node_->subscribe("/spencer/perception_internal/detected_persons/pre/velodyne_fast", 10, &DetectionCom::laser_3d_fast_cb, this);
    cam_front_sub = node_->subscribe("/spencer/perception_internal/detected_persons/pre/rgbd_front_top/upper_body", 10, &DetectionCom::cam_front_cb, this);
    cam_back_sub = node_->subscribe("/spencer/perception_internal/detected_persons/pre/rgbd_rear_top/upper_body", 10, &DetectionCom::cam_back_cb, this);
    laser_2d_sub = node_->subscribe("/spencer/perception_internal/detected_persons/pre/laser_front", 10, &DetectionCom::laser_2d_cb, this);


    // === SERVICE CLIENTS ===


    // === TIMER ===
    my_timer_ = node_->createTimer(ros::Duration(0.033), &DetectionCom::timerCallback, this);

    // === Transform msgs to base_link ===


ros::Duration(5.0).sleep();


}

//########## CALLBACK: SUBSCRIBER ######################################################################################
void DetectionCom::laser_3d_cb(const sg_msgs::DetectedPersonsOdomConstPtr &msg)
{
  if(!velo_pushed && !msg->DetectedPersons.detections.empty())
     {

      {
             boost::mutex::scoped_lock lock(mutex);
              global_detections_odom_3D=*msg;
              velo_pushed=true;
       }



     }

}

void DetectionCom::laser_3d_fast_cb(const sg_msgs::DetectedPersonsOdomConstPtr &msg)
{
  if(!velo_fast_pushed && !msg->DetectedPersons.detections.empty())
  {

   {
          boost::mutex::scoped_lock lock(mutex);
           global_detections_odom_3D_fast=*msg;
           velo_fast_pushed=true;
    }

  }
}

void DetectionCom::cam_front_cb(const sg_msgs::DetectedPersonsOdomConstPtr &msg)
{
  if(!cam_front_pushed && !msg->DetectedPersons.detections.empty())
  {


        {
               boost::mutex::scoped_lock lock(mutex);
                global_detections_odom_cam_front=*msg;
                cam_front_pushed=true;
         }


  }

}


void DetectionCom::cam_back_cb(const sg_msgs::DetectedPersonsOdomConstPtr &msg)
{
  if(!cam_back_pushed && !msg->DetectedPersons.detections.empty())
  {

   {
          boost::mutex::scoped_lock lock(mutex);
           global_detections_odom_cam_back=*msg;
           cam_back_pushed=true;
    }


  }
}

void DetectionCom::laser_2d_cb(const sg_msgs::DetectedPersonsOdomConstPtr &msg)
{

    if(!laser2d_pushed && !msg->DetectedPersons.detections.empty())
    {
           boost::mutex::scoped_lock lock(mutex);
            global_detections_odom_2D=*msg;
             laser2d_pushed=true;
      }




}


//########## CALLBACK: TIMER ###########################################################################################
void DetectionCom::timerCallback(const ros::TimerEvent &evt)
{

  tf::StampedTransform trans_base;
  try
  {

    listener.lookupTransform("base_link","odom",ros::Time(0),trans_base);

  }
  catch(ros::Exception e)
  {
    ROS_ERROR("In Finale transformation: %s",e.what());
  }
  catch(tf::LookupException e)
  {
    ROS_ERROR("In Finale LookupException: %s",e.what());
  }
  spencer_tracking_msgs::DetectedPersons detected_ps;
  detected_ps.header.stamp=ros::Time::now();
  detected_ps.header.frame_id="base_link";

      for (auto &detection:global_detections_odom_3D.DetectedPersons.detections) {
        tf::Transform pose_tf;
        tf::StampedTransform tf_odom;
        tf::poseMsgToTF(detection.pose.pose,pose_tf);
        tf::transformStampedMsgToTF(global_detections_odom_3D.Transform,tf_odom);
        tf::poseTFToMsg(trans_base*tf_odom*pose_tf,detection.pose.pose);
        detected_ps.detections.push_back(detection);
      }

      for (auto &detection:global_detections_odom_3D_fast.DetectedPersons.detections) {
        tf::Transform pose_tf;
        tf::StampedTransform tf_odom;
        tf::poseMsgToTF(detection.pose.pose,pose_tf);
        tf::transformStampedMsgToTF(global_detections_odom_3D.Transform,tf_odom);
        tf::poseTFToMsg(trans_base*tf_odom*pose_tf,detection.pose.pose);
        detected_ps.detections.push_back(detection);
      }
    pub_3D.publish(detected_ps);
    global_detections_odom_3D.DetectedPersons.detections.clear();
    global_detections_odom_3D_fast.DetectedPersons.detections.clear();

    for (auto &detection:global_detections_odom_cam_front.DetectedPersons.detections) {
      tf::Transform pose_tf;
      tf::StampedTransform tf_odom;
      tf::poseMsgToTF(detection.pose.pose,pose_tf);
      tf::transformStampedMsgToTF(global_detections_odom_cam_front.Transform,tf_odom);
      tf::poseTFToMsg(trans_base*tf_odom*pose_tf,detection.pose.pose);
    }

  spencer_tracking_msgs::DetectedPersons detected_ps_cam_front;
  detected_ps_cam_front.header.stamp=ros::Time::now();
  detected_ps_cam_front.header.frame_id="base_link";
  detected_ps_cam_front.detections=global_detections_odom_cam_front.DetectedPersons.detections;
  pub_cam_front.publish(detected_ps_cam_front);
  global_detections_odom_cam_front.DetectedPersons.detections.clear();

  for (auto &detection:global_detections_odom_cam_back.DetectedPersons.detections) {
    tf::Transform pose_tf;
    tf::StampedTransform tf_odom;
    tf::poseMsgToTF(detection.pose.pose,pose_tf);
    tf::transformStampedMsgToTF(global_detections_odom_cam_back.Transform,tf_odom);
    tf::poseTFToMsg(trans_base*tf_odom*pose_tf,detection.pose.pose);
  }

spencer_tracking_msgs::DetectedPersons detected_ps_cam_back;
detected_ps_cam_back.header.stamp=ros::Time::now();
detected_ps_cam_back.header.frame_id="base_link";
detected_ps_cam_back.detections=global_detections_odom_cam_back.DetectedPersons.detections;
pub_cam_back.publish(detected_ps_cam_back);
global_detections_odom_cam_back.DetectedPersons.detections.clear();



//    spencer_tracking_msgs::DetectedPersons temp_detections;
//    {
//      boost::mutex::scoped_lock lock(mutex);
//        temp_detections=global_detections;
//        global_detections.detections.clear();
//      }
//    temp_detections.header.stamp=ros::Time::now();
//    temp_detections.header.frame_id="base_link";
//    spencer_pub.publish(temp_detections);
    velo_pushed=false;
    cam_front_pushed=false;
    cam_back_pushed=false;
    velo_fast_pushed=false;
    laser2d_pushed=false;




}

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{

    ros::init(argc, argv, "vis_path");

    ros::NodeHandle node_handle;
    DetectionCom detection_com(node_handle);


     ROS_INFO("Node is spinning...");
     ros::spin();

     return 0;






}
