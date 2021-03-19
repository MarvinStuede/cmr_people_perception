/* *****************************************************************
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the GPL-style license found in the
LICENSE file in the root directory of this source tree.
*/
/**
 * @file   %FILENAME%
 * @author %USER% (%$EMAIL%)
 * @date   %DATE%
 *
 * @brief  Filedescription
 */

#ifndef ROTATE_IMAGE_PET_ROTATE_IMAGE_PET_H
#define ROTATE_IMAGE_PET_ROTATE_IMAGE_PET_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <message_filters/subscriber.h>
 #include <message_filters/synchronizer.h>
 #include <message_filters/sync_policies/approximate_time.h>
 #include <sensor_msgs/Image.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_tracking_msgs/DetectedPersons.h>
#include <people_msgs/People.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmr_msgs/EyeFollowerPersons.h>
#include <math.h>
#include <string>
#include <iostream>


class MessageRepublisher
{
public:
    MessageRepublisher(ros::NodeHandle &node_handle);

private:
    // node handle
    ros::NodeHandle *node_;

    // ros communication
    ros::Subscriber sub_spencer;
    ros::Publisher my_publisher_;
    ros::ServiceServer my_service_server_;
    ros::ServiceClient my_service_client_;
    ros::Timer my_timer_;
    ros::Subscriber sub_pointcloud;
    ros::Subscriber sub_detection;

    ros::Publisher pub_people;
    ros::Publisher pub_follower;
    ros::Publisher pub_follower_detection;

    // parameters
    double my_double_parameter_;
    std::string my_string_parameter_;
    int my_int_parameter_;
    bool my_bool_parameter_;

    // callbacks
    void subscriberCallback(const spencer_tracking_msgs::TrackedPersons &spencer_persons);
    void pointcloudcb(const sensor_msgs::PointCloudConstPtr &pointcloud);
    void detectCallback(const spencer_tracking_msgs::DetectedPersons &detections);

    bool serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void timerCallback(const ros::TimerEvent &evt);
};

#endif // ROTATE_IMAGE_PET_ROTATE_IMAGE_PET_H
