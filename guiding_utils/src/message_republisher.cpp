/* *****************************************************************
 *
 * rotate_image_pet
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

/**
 * @file   %FILENAME%
 * @author %USER% (%$EMAIL%)
 * @date   %DATE%
 *
 * @brief  Filedescription
 */

#include "rotate_image_pet/message_republisher.h"

//########## CONSTRUCTOR ###############################################################################################
MessageRepublisher::MessageRepublisher(ros::NodeHandle &node_handle):
    node_(&node_handle)
{

    // === PARAMETERS ===
    node_->param("rotate_image_pet/my_double_param", my_double_parameter_, 0.0);
    node_->param("rotate_image_pet/my_string_param", my_string_parameter_, std::string(""));
    node_->param("rotate_image_pet/my_int_param", my_int_parameter_, 0);
    node_->param("rotate_image_pet/my_bool_param", my_bool_parameter_, false);

    // === SUBSCRIBERS ===
    sub_spencer = node_->subscribe("/spencer/perception/tracked_persons", 10, &MessageRepublisher::subscriberCallback, this);
    sub_detection=node_->subscribe("/spencer/perception/detected_persons",10, &MessageRepublisher::detectCallback, this);


    // === PUBLISHERS ===
    my_publisher_ = node_->advertise<std_msgs::String>("rotate_image_pet/my_topic", 10);
    pub_people=node_->advertise<people_msgs::People>("/people",10);
    pub_follower_detection=node_->advertise<cmr_msgs::EyeFollowerPersons>("/sobi/eye_follower_persons",10);

    // === SERVICE SERVERS ===
    my_service_server_ = node_->advertiseService("rotate_image_pet/my_service", &MessageRepublisher::serviceCallback, this);

    // === SERVICE CLIENTS ===
    my_service_client_ = node_->serviceClient<std_srvs::Empty>("some_service");

    // === TIMER ===
    my_timer_ = node_->createTimer(ros::Duration(0.1), &MessageRepublisher::timerCallback, this);
}


//########## CALLBACK: SUBSCRIBER ######################################################################################
void MessageRepublisher::subscriberCallback(const spencer_tracking_msgs::TrackedPersons &spencer_persons)
{
  people_msgs::People people;
  cmr_msgs::EyeFollowerPersons follower_persons;
  people.header=spencer_persons.header;
  follower_persons.header=spencer_persons.header;
  if(spencer_persons.tracks.size())
  {
  for (int i=0;i<spencer_persons.tracks.size();i++) {
      people_msgs::Person person;
      cmr_msgs::EyeFollowerPerson follower_person;
      person.position=spencer_persons.tracks[i].pose.pose.position;
      person.velocity.x=spencer_persons.tracks[i].twist.twist.linear.x;
      person.velocity.y=spencer_persons.tracks[i].twist.twist.linear.y;
      person.name=std::to_string(spencer_persons.tracks[i].detection_id);
      people.people.push_back(person);

      follower_person.track_id=spencer_persons.tracks[i].track_id;
      follower_person.is_matched=spencer_persons.tracks[i].is_matched;
      follower_person.is_occluded=spencer_persons.tracks[i].is_occluded;
      follower_person.distance=sqrt(pow(spencer_persons.tracks[i].pose.pose.position.x,2)+pow(spencer_persons.tracks[i].pose.pose.position.y,2));
      follower_person.angle=atan2(spencer_persons.tracks[i].pose.pose.position.y,spencer_persons.tracks[i].pose.pose.position.x);
  }

  }
  pub_people.publish(people);
}

void MessageRepublisher::pointcloudcb(const sensor_msgs::PointCloudConstPtr &cloud)
{


}

void MessageRepublisher::detectCallback(const spencer_tracking_msgs::DetectedPersons &detections)
{
    cmr_msgs::EyeFollowerPersons follower_persons;
    follower_persons.header=detections.header;

   for(spencer_tracking_msgs::DetectedPerson detection : detections.detections)
   {   
     
       cmr_msgs::EyeFollowerPerson follower_person;
       size_t found1 =detection.modality.find("laser3d");
       size_t found2 =detection.modality.find("laser3d");
       if(found1 != std::string::npos && found2 != std::string::npos)
       {
 
       follower_person.distance=sqrt(pow(detection.pose.pose.position.x,2)+pow(detection.pose.pose.position.y,2));
       follower_person.angle=atan2(detection.pose.pose.position.y,detection.pose.pose.position.x);
       follower_persons.tracks.push_back(follower_person);
       }
   }
   pub_follower_detection.publish(follower_persons);
}

//########## CALLBACK: SERVICE SERVER ##################################################################################
bool MessageRepublisher::serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Service call received.");
    return true;
}

//########## CALLBACK: TIMER ###########################################################################################
void MessageRepublisher::timerCallback(const ros::TimerEvent &evt)
{
    // === PUBLISH A MESSAGE ===
    std_msgs::String msg;
    msg.data = "Hello World.";
    my_publisher_.publish(msg);

    // === CALL A SERVICE ===
    std_srvs::Empty srv;
    my_service_client_.call(srv);
}

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "message_republisher");

    ros::NodeHandle node_handle;
    MessageRepublisher message_republisher(node_handle);

    ROS_INFO("Node is spinning...");
    ros::spin();

    return 0;
}
