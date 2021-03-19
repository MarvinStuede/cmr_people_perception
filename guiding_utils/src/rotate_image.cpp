#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


 
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub_image;
ros::Publisher pub_cam_info;
 void callback(const ImageConstPtr& image1, const CameraInfoConstPtr & cam_info)
 {

     cv_bridge::CvImagePtr cv_ptr;
          try
           {
           cv_ptr = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::RGB8);
          }
         catch (cv_bridge::Exception& e)
          {
            ROS_ERROR("cv_bridge exception: %s", e.what());
             return;
  
     }
     cv::Mat src = cv_ptr->image;

     double angle = 90;
     // get rotation matrix for rotating the image around its center in pixel coordinates
      cv::Point2f center((src.cols-1)/2.0, (src.rows-1)/2.0);
      cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
      // determine bounding rectangle, center not relevant
      cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();
      // adjust transformation matrix
      rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
      rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;
      cv::Mat dst;
       cv::warpAffine(src, dst, rot, bbox.size());
       cv_bridge::CvImage out_msg;
       out_msg.header   = image1->header; // Same timestamp and tf frame as input image
       out_msg.encoding = image1->encoding;
       out_msg.image    = dst; // Your cv::Mat
       
       CameraInfo cam_info_rot=*cam_info;
       cam_info_rot.header.frame_id="cam_back_rotated_color_optical_frame";
       cam_info_rot.K.at(0)=cam_info->K.at(4);
       cam_info_rot.K.at(2)=cam_info->K.at(5);
       cam_info_rot.K.at(5)=cam_info->K.at(2);
       cam_info_rot.K.at(4)=cam_info->K.at(0);
       cam_info_rot.height=cam_info->width;
       cam_info_rot.width=cam_info->height;

       pub_image.publish(out_msg.toImageMsg());
       pub_cam_info.publish(cam_info_rot);
 }
 
 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "vision_node");
 
   ros::NodeHandle nh;
   message_filters::Subscriber<Image> image1_sub(nh, "/cam_back/color/image_raw", 1);
   message_filters::Subscriber<CameraInfo> image2_sub(nh, "/cam_back/color/camera_info", 1);
 
   typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
   // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
   Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
   sync.registerCallback(boost::bind(&callback, _1, _2));
   
   pub_image = nh.advertise<Image>("cam_back_rotated/color/image_raw", 10);
   pub_cam_info=nh.advertise<CameraInfo>("cam_back_rotated/color/camera_info",10);
 
   ros::spin();
 
   return 0;
 }
