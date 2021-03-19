/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/

/*!
  *\file: cluster_polygon.h
  * \author: Alexander Petersen
  * \date: 21.11.19
  * \brief: ClusterPolygon Classe:
  * Class to make interactive Polygone
  */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "yaml-cpp/yaml.h"
#include "fstream"
#include "yaml-cpp/ostream_wrapper.h"
#include <ros/package.h>
#include <sg_msgs/PolygonArray.h>


#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

/// Menuhandler for menu on points
interactive_markers::MenuHandler menu_handler;

/// Server for taking interactions
std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

/// Points for Polygon
std::vector<geometry_msgs::Point32> _points;

/// Array for Polygon
std::vector<std::vector<geometry_msgs::Point32>> _points_array;

/// sets the number of current polygon to first one
int current_poly=0;

/** \brief Creating new point for Polygon
 * \param fixed
 * \parm interaction_mode
 * \param position
 * \param show_6dof
 * \param name
 */
void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof ,std::string name);

using namespace visualization_msgs;
class ClusterPolygon
{
public:
     /// Creating Clusterpolygon Class
    ClusterPolygon(ros::NodeHandle &node_handle);

private:
    /// node handle
    ros::NodeHandle *node_;

    /// ros communication
    ros::Subscriber my_subscriber_;
    ros::Publisher my_publisher_;
     ros::Publisher my_publisher2_;
      ros::Publisher my_publisher3_;
       ros::Publisher my_publisher4_;
    ros::ServiceServer my_service_server_;
    ros::ServiceClient my_service_client_;

    /// Creating Timer
    ros::Timer my_timer_;

    /// Creating polygon
    geometry_msgs::PolygonStamped polygon;

    /// Counts points of Polygon
    int count_points=0;


    /// parameters for Config file
    double my_double_parameter_;
    std::string my_string_parameter_;
    int my_int_parameter_;
    bool my_bool_parameter_;


    /// Callbacks
    void subscriberCallback(const geometry_msgs::PointStamped &msg);
    bool serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void timerCallback(const ros::TimerEvent &evt);

};


/** \brief Creating a Box
 * \param msg Message with scale information to create Box
 */
 Marker makeBox( InteractiveMarker &msg )
 {


   Marker marker;

   marker.type = Marker::SPHERE;
   marker.scale.x = msg.scale * 1.0;
   marker.scale.y = msg.scale * 1.0;
   marker.scale.z = msg.scale * 1.0;
   marker.color.r = 255;
   marker.color.g = 0.0;
   marker.color.b = 0.0;
   marker.color.a = 1.0;

   return marker;
 }


 /** \brief Controlling feedback from user to interacte with
  * \param feedback Feedback from user
  */
 void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
 {

   // Information about Feedback
   std::ostringstream s;
   s << "Feedback from marker '" << feedback->marker_name << "' "
       << " / control '" << feedback->control_name << "'";

   std::ostringstream mouse_point_ss;
   if( feedback->mouse_point_valid )
   {
     mouse_point_ss << " at " << feedback->mouse_point.x
                    << ", " << feedback->mouse_point.y
                    << ", " << feedback->mouse_point.z
                    << " in frame " << feedback->header.frame_id;
   }

   //Eventhandler for different aktions
   switch ( feedback->event_type )
   {

     // Case: Interaktion for a simple button click -> Only output
     case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
       ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
       break;

    // Case: Menu handler (for example: switching points, add new polygon, delete polygon)
     case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
   {
       ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );

       //************ Delete **********************
       // Deleting a Point from Polygon
         if(feedback->menu_entry_id ==1)
         {
           int point_num =std::stoi(feedback->marker_name);

          _points_array.at(current_poly).erase(_points_array.at(current_poly).begin()+point_num);  //deleting
           server->clear();  //clearing all points

           // Rewrite Polygon
           int i=0;
           for (auto point :_points_array.at(current_poly)) {

              tf::Vector3 position;
              position = tf::Vector3( 0,point.y,-point.x);
              make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_PLANE, position, false, std::to_string(i));
              i++;

             }
            }

         //************ Left **********************
         // Switching to left point from Polygon
         if(feedback->menu_entry_id ==3)
         {
           int point_num =std::stoi(feedback->marker_name);
           std::vector<geometry_msgs::Point32> temp_points;

           for (auto it=_points_array.at(current_poly).begin()+point_num;it<_points_array.at(current_poly).end();it++) {
             temp_points.push_back(*it);
             }
           int end=_points_array.at(current_poly).size()-point_num;
           for (auto it=_points_array.at(current_poly).begin();it<_points_array.at(current_poly).end()-end;it++) {
             temp_points.push_back(*it);
             }

           _points_array.at(current_poly)=temp_points;
           server->clear(); //clearing all points
           int i=0;
           // Rewrite Polygon
           for (auto point :_points_array.at(current_poly)) {

             tf::Vector3 position;
              position = tf::Vector3( 0,point.y,-point.x);
            make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_PLANE, position, false, std::to_string(i));
            i++;

             }
           }
          //************ Right **********************
         // Switching to right point from Polygon
         if(feedback->menu_entry_id ==4)
         {
           int point_num =std::stoi(feedback->marker_name);
           std::vector<geometry_msgs::Point32> temp_points;
           int count=0;
           for (auto it=_points_array.at(current_poly).begin()+point_num+1;it<_points_array.at(current_poly).end();it++) {
             temp_points.push_back(*it);
             count++;
             }

           for (auto it=_points_array.at(current_poly).begin();it<_points_array.at(current_poly).end()-count;it++) {
             temp_points.push_back(*it);
             }

           _points_array.at(current_poly)=temp_points;
           server->clear();   //clearing all points
           int i=0;
           // Rewrite Polygon
           for (auto point :_points_array.at(current_poly)) {

             tf::Vector3 position;
              position = tf::Vector3( 0,point.y,-point.x);
            make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_PLANE, position, false, std::to_string(i));
            i++;

             }
           }


         //************ save **********************
         // Saving polygon to a YAML FILE
         if(feedback->menu_entry_id ==6)
         {
             // Controlls if Polygon is over 2 Points
             if(_points_array.at(current_poly).size()>2)
             {
             std::ofstream ofile;
             std::string save_path=ros::package::getPath("cluster_polygon");
             std::string save_file ="/params/polygon";
             std::string save_no=std::to_string(ros::Time::now().toNSec());
             std::string save_end=".yaml";
             ofile.open(save_path +save_file+save_no+save_end,std::ios::out);
             ofile << "points: [";
             int i=0;
             for(auto &val :_points_array.at(current_poly))
             {
               if(i<_points_array.at(current_poly).size()-1)
               ofile <<"["<<val.x<<","<<val.y<<"]"<<",";
               else
                 ofile <<"["<<val.x<<","<<val.y<<"]";


               i++;
             }
              ofile <<"]";
             ofile.close();
              }
             else {
               std::cout<<"Nothing to save - to few datas"<<std::endl;
             }


            }
      //************ New **********************
      // Deleting whole Polygon
         if(feedback->menu_entry_id ==8)
         {
           _points_array.at(current_poly).clear();

           server->clear();

           }

    //************ change Polygon **********************
    // switch to next Polygon

         if(feedback->menu_entry_id ==9)
         {


            server->clear(); //clearing all points

            // changing polygon
            if(current_poly==_points_array.size()-1)
            {

               current_poly=0;

            }
            else {

              current_poly++;
            }

            int i=0;
            // rewrite polygon
            for (auto point :_points_array.at(current_poly)) {

              tf::Vector3 position;
               position = tf::Vector3( 0,point.y,-point.x);
             make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_PLANE, position, false, std::to_string(i));
             i++;

              }

           }
  //************ Add Polygon **********************
  // Adds new Polygon
         if(feedback->menu_entry_id ==10)
         {
            // adding polygon to list
           _points_array.push_back(_points);
            server->clear();  // clearing polygon
           current_poly=_points_array.size()-1;


           }
       server->applyChanges();

       break;
     }

     // Case: Changing position of point
     case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
   {
     // Output Console
       ROS_INFO_STREAM( s.str() << ": pose changed"
           << "\nposition = "
           << feedback->pose.position.x
           << ", " << feedback->pose.position.y
           << ", " << feedback->pose.position.z
           << "\norientation = "
           << feedback->pose.orientation.w
           << ", " << feedback->pose.orientation.x
           << ", " << feedback->pose.orientation.y
           << ", " << feedback->pose.orientation.z
           << "\nframe: " << feedback->header.frame_id
           << " time: " << feedback->header.stamp.sec << "sec, "
           << feedback->header.stamp.nsec << " nsec" );
         int point_num=std::stoi(feedback->marker_name);

       // changing planar position of point
       _points_array.at(current_poly).at(point_num).x=-feedback->pose.position.z;
       _points_array.at(current_poly).at(point_num).y=feedback->pose.position.y;

       break;
}

     // Case: Mouse down -> Only Output
     case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
       ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
       break;

     // Case: Mouse up -> Only Output
     case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
       ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
       break;
   }

   server->applyChanges();  //call to apply all changes
 }

 /** \brief Define Control Modus of point
  * \param msg Messagetyp of interactive Marker
  * \return InteractiveMarkerControl Type of Control
  */
 InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
 {
   InteractiveMarkerControl control;
   control.always_visible = true;
   control.markers.push_back( makeBox(msg) );
   msg.controls.push_back( control );

   return msg.controls.back();
 }



 void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof ,std::string name)
 {
   InteractiveMarker int_marker;
   int_marker.header.frame_id = "base_link_r";
   tf::pointTFToMsg(position, int_marker.pose.position);
   int_marker.scale = 0.1;

   int_marker.name = name;
   int_marker.description = "Simple 6-DOF Control";

   // insert a box
   makeBoxControl(int_marker);
   int_marker.controls[0].interaction_mode = interaction_mode;
  int_marker.controls[0].orientation_mode=1;

   int_marker.description = std::string(name) ;

   server->insert(int_marker);
   server->setCallback(int_marker.name, &processFeedback);
   if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
     menu_handler.apply( *server, int_marker.name );
 }



//########## CONSTRUCTOR ###############################################################################################
ClusterPolygon::ClusterPolygon(ros::NodeHandle &node_handle):
    node_(&node_handle)
{
    // === PARAMETERS ===
    node_->param("cluster_polygon/my_double_param", my_double_parameter_, 0.0);
    node_->param("cluster_polygon/my_string_param", my_string_parameter_, std::string(""));
    node_->param("cluster_polygon/my_int_param", my_int_parameter_, 0);
    node_->param("cluster_polygon/my_bool_param", my_bool_parameter_, false);

    // === SUBSCRIBERS ===
    my_subscriber_ = node_->subscribe("/clicked_point", 10, &ClusterPolygon::subscriberCallback, this);

    // === PUBLISHERS ===
    my_publisher_ = node_->advertise<geometry_msgs::PolygonStamped>("/cluster_polygon", 10);
        my_publisher2_ = node_->advertise<geometry_msgs::PolygonStamped>("/cluster_polygon2", 10);
            my_publisher3_ = node_->advertise<geometry_msgs::PolygonStamped>("/cluster_polygon3", 10);
                my_publisher4_ = node_->advertise<geometry_msgs::PolygonStamped>("/cluster_polygon4", 10);

    // === SERVICE SERVERS ===
    my_service_server_ = node_->advertiseService("cluster_polygon/my_service", &ClusterPolygon::serviceCallback, this);

    // === SERVICE CLIENTS ===
    my_service_client_ = node_->serviceClient<std_srvs::Empty>("some_service");

    // === TIMER ===
    my_timer_ = node_->createTimer(ros::Duration(0.1), &ClusterPolygon::timerCallback, this);


}

//########## CALLBACK: SUBSCRIBER ######################################################################################
// Callback for adding new Points in RVIZ
void ClusterPolygon::subscriberCallback(const geometry_msgs::PointStamped &msg)
{
  if(msg.header.frame_id=="base_link")
  {
    geometry_msgs::Point32 temp_point;
    temp_point.x=msg.point.x;
    temp_point.y=msg.point.y;
    temp_point.z=0.0;

    tf::Vector3 position;
     position = tf::Vector3( 0,msg.point.y,-msg.point.x);
   make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_PLANE, position, false, std::to_string(_points_array.at(current_poly).size()));
   server->applyChanges();
  _points_array.at(current_poly).push_back(temp_point);


  }
  else {

    ROS_INFO("Points is not in base_link. Change in RVIZ fixe frame to base_link in gloabal options");
  }
}

//########## CALLBACK: SERVICE SERVER ##################################################################################
bool ClusterPolygon::serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Service call received.");
    return true;
}

//########## CALLBACK: TIMER ###########################################################################################
// Timer to publish Polygons to apply changes
void ClusterPolygon::timerCallback(const ros::TimerEvent &evt)
{

    // Publishing first Polygon
    geometry_msgs::PolygonStamped poly;
    poly.header.stamp=ros::Time::now();
    poly.header.frame_id="base_link";
  for (auto point :_points_array.at(current_poly)) {
    poly.polygon.points.push_back(point);
  }
  my_publisher_.publish(poly);

  // Publishing second Polygon
  if(_points_array.size()>1)
  {
  geometry_msgs::PolygonStamped poly2;
  poly2.header.stamp=ros::Time::now();
  poly2.header.frame_id="base_link";
  int num_poly=current_poly+1;
  if(num_poly>_points_array.size()-1)
    num_poly=num_poly-_points_array.size();
   for (auto point :_points_array.at(num_poly)) {
       poly2.polygon.points.push_back(point);
             }
  my_publisher2_.publish(poly2);
  }

  // Publishing 3. Polygon
  if(_points_array.size()>2)
  {
  geometry_msgs::PolygonStamped poly3;
  poly3.header.stamp=ros::Time::now();
  poly3.header.frame_id="base_link";
  int num_poly=current_poly+2;
  if(num_poly>_points_array.size()-1)
    num_poly=num_poly-_points_array.size();
  for (auto point :_points_array.at(num_poly)) {
     poly3.polygon.points.push_back(point);
      }
   my_publisher3_.publish(poly3);
  }

  // Publishing 4. Polygon
  if(_points_array.size()>3)
  {
  geometry_msgs::PolygonStamped poly4;
  poly4.header.stamp=ros::Time::now();
  poly4.header.frame_id="base_link";
  int num_poly=current_poly+3;
  if(num_poly>_points_array.size()-1)
    num_poly=num_poly-_points_array.size();
  for (auto point :_points_array.at(num_poly)) {
    poly4.polygon.points.push_back(point);
     }
  my_publisher4_.publish(poly4);
  }
}



//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{

  _points_array.push_back(_points);
    ros::init(argc, argv, "cluster_polygon");    // init node

    ros::NodeHandle node_handle;

    ClusterPolygon cluster_polygon(node_handle);   // creating Class

   server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );    //  reseting server

    ros::Duration(0.1).sleep();


      // Creating Menu Handler
      menu_handler.insert( "Delete", &processFeedback );    // Deleting points from polygon

      interactive_markers::MenuHandler::EntryHandle sub_menu_handle =menu_handler.insert( "Insert" );  // Inserts left or right from point a new point
      menu_handler.insert( sub_menu_handle, "Left", &processFeedback );
      menu_handler.insert( sub_menu_handle, "Right", &processFeedback );

      interactive_markers::MenuHandler::EntryHandle sub_menu_handle2=menu_handler.insert( "Save" );  // Saving Polygon to a yaml file
      menu_handler.insert( sub_menu_handle2, "Sure?", &processFeedback );

      interactive_markers::MenuHandler::EntryHandle sub_menu_handle3=menu_handler.insert( "New" );   // Deleting all points from current polygon
      menu_handler.insert( sub_menu_handle3, "Sure?", &processFeedback );

       menu_handler.insert( "Change Polygon", &processFeedback );  // changing polygon

       menu_handler.insert( "Add Polygon", &processFeedback );   // add new polygon


    ROS_INFO("Node is spinning...");
    ros::spin();
    server.reset();
    return 0;
}
