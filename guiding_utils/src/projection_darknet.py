#!/usr/bin/env python
"""
A ROS node to get 3D values of bounding boxes with median filter

This node gets the bounding boxes and gets the real world coordinates of
them by using depth values. It simply gets the x and y values of center point
and gets the median value of depth values as z value.

"""

import rospy

import message_filters

import numpy as np

from cv_bridge import CvBridge

from sensor_msgs.msg import Image

from cob_perception_msgs.msg import DetectionArray

from spencer_tracking_msgs.msg import DetectedPersons , DetectedPerson

from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox

from sg_msgs.msg import DetectedPersonsOdom

import tf

from geometry_msgs.msg import TransformStamped

import threading

class ProjectionNode(object):
    """Get 3D values of bounding boxes returned by node.

    _bridge (CvBridge): Bridge between ROS and CV image
    pub (Publisher): Publisher object for face depth results
    f (Float): Focal Length
    cx (Int): Principle Point Horizontal
    cy (Int): Principle Point Vertical


    """
    def __init__(self):
        super(ProjectionNode, self).__init__()

        # init the node
        rospy.init_node('projection_node', anonymous=False)

        self.listener=tf.TransformListener()
        
        self._bridge = CvBridge()

        (depth_topic, face_topic, output_topic, f, cx, cy,id_increment,id_offset,output_spencer) = \
            self.get_parameters()

         # Subscribe to the face positions
        sub_obj = message_filters.Subscriber(face_topic,\
            BoundingBoxes)

        sub_depth = message_filters.Subscriber(depth_topic,\
            Image)
        
        # Advertise the result of Face Depths
        self.pub = rospy.Publisher(output_topic, \
            DetectionArray, queue_size=1)
        self.pub_pic=rospy.Publisher("/darknet_projection/image",Image,queue_size=1)
        self.pub_spencer=rospy.Publisher("/test_spencer",DetectedPersons,queue_size=1)
        #self.pub_spencer=rospy.Publisher("", DetectedPersons,queue_size=1)
        self.pub_spencer_odom=rospy.Publisher(output_spencer,DetectedPersonsOdom,queue_size=1)
        # Create the message filter
        ts = message_filters.ApproximateTimeSynchronizer(\
            [sub_obj, sub_depth], \
            2, \
            0.9)

        ts.registerCallback(self.detection_callback)

        self.timer = rospy.Timer(rospy.Duration(0.166), self.timer_cb)
        self.timer_pros=rospy.Timer(rospy.Duration(0.02),self.timer_cb_pros)
        self.new_msgs=False
        self.f = f
        self.cx = cx
        self.cy = cy

        self.detection_id_increment=id_increment
        self.detection_id_offset=id_offset
        self.current_detection_id=self.detection_id_offset
        self.spencer_detect = DetectedPersons()
        self.last_time=rospy.get_rostime().secs
        self.bridge = CvBridge()
        self.new_bounding=False
        self.message=[]
        self.lock=threading.RLock()

        # spin
        rospy.spin()


    def shutdown(self):
        """
        Shuts down the node
        """
        rospy.signal_shutdown("See ya!")

    def timer_cb_pros(self,timer):
        if(self.new_bounding):
            self.lock.acquire()
            self.new_bounding=False
            msg=self.message[0]
            depth=self.message[1]
            self.lock.release()
            t=TransformStamped()
            try:
                #print(rospy.Time.now().to_sec)
                #print(depth.header.stamp)
                #time=rospy.Time.now().to_sec-depth.header.stamp.secs
                #print(time)
                (trans,rot)=self.listener.lookupTransform("odom",depth.header.frame_id,rospy.Time(0))
                
                t.header.stamp=rospy.Time.now()
                t.header.frame_id="odom"
                t.child_frame_id=depth.header.frame_id
                t.transform.translation.x=trans[0]
                t.transform.translation.y=trans[1]
                t.transform.translation.z=trans[2]
                t.transform.rotation.x=rot[0]
                t.transform.rotation.y=rot[1]
                t.transform.rotation.z=rot[2]
                t.transform.rotation.w=rot[3]
            except(tf.LookupException):
                print("LookupException")
            except(tf.ConnectivityException):
                print("ConnectivityException")
            except(tf.ExtrapolationException):
                print("ExtrapolationException")
                
            
            cv_depth = self._bridge.imgmsg_to_cv2(depth, "passthrough")
            #cv_color = self._bridge.imgmsg_to_cv2(color, "rgb8")

            # get the number of detections
            no_of_detections = len(msg.bounding_boxes)
            spencer_detected_persons = DetectedPersons()
            spencer_detected_persons.header=msg.image_header
            # Check if there is a detection
            if no_of_detections > 0:
                for i, boxes in enumerate(msg.bounding_boxes):
                    
                    x =  boxes.xmin
                    y =  boxes.ymin
                    xmax =  boxes.xmax
                    ymax = boxes.ymax
                    width=xmax-x
                    height=ymax-y
                    scale=0.6
                    height_delta=(height-height*scale)/2
                    
                    
                    width_delta =(width-width*scale)/2

                    cv_depth_bounding_box = cv_depth[int(y+height_delta):int(y+height_delta+height*scale),int(x+width_delta):int(x+width_delta+width*scale)]
                    #cv_color_debug=cv_color[int(y+height_delta):int(y+height_delta+height*scale),int(x+width_delta):int(x+width_delta+width*scale)]
                    try:

                        depth_mean = np.nanmedian(\
                        cv_depth_bounding_box[np.nonzero(cv_depth_bounding_box)])
                    
                        real_x = (x + width/2-self.cx)*(depth_mean)/self.f

                        real_y = (y + height/2-self.cy)*(depth_mean)/self.f


                        if(boxes.Class == 'person'):
                            spencer_detected_person=DetectedPerson()
                            spencer_detected_person.modality=spencer_detected_person.MODALITY_GENERIC_RGBD
                            spencer_detected_person.confidence=1
                            spencer_detected_person.pose.pose.position.x = real_x
                            spencer_detected_person.pose.pose.position.y = real_y
                            spencer_detected_person.pose.pose.position.z = depth_mean
                            spencer_detected_person.pose.pose.orientation.w = 1.0
                            LARGE_VARIANCE = 999999999
                            pose_variance=0.05
                            spencer_detected_person.pose.covariance[0*6+0] =pose_variance
                            spencer_detected_person.pose.covariance[1*6+1] =pose_variance
                            spencer_detected_person.pose.covariance[2*6+2] =pose_variance
                            spencer_detected_person.pose.covariance[3*6+3] =LARGE_VARIANCE
                            spencer_detected_person.pose.covariance[4*6+4] =LARGE_VARIANCE
                            spencer_detected_person.pose.covariance[5*6+5] =LARGE_VARIANCE
                            spencer_detected_person.detection_id=self.current_detection_id
                            self.current_detection_id=self.current_detection_id+self.detection_id_increment
                            spencer_detected_persons.detections.append(spencer_detected_person)
                            #self.pub_pic.publish(self.bridge.cv2_to_imgmsg(cv_color_debug,  "rgb8"))
            
                    except Exception as e:
                        print e
            self.new_msgs=True
            self.last_time=rospy.get_rostime().secs
            self.spencer_detect=spencer_detected_persons
            spencer_detect_odom=DetectedPersonsOdom()
            spencer_detect_odom.DetectedPersons=spencer_detected_persons
            spencer_detect_odom.Transform=t
            #self.pub_spencer.publish(spencer_detected_persons)
            self.pub_spencer_odom.publish(spencer_detect_odom)

    def timer_cb(self,timer):
        """
        Timercallback: To publish spencer msgs

        """
        if((rospy.get_rostime().secs-self.last_time)>0.05):
           self.new_msgs=False
        if(self.new_msgs):
            #self.pub_spencer.publish(self.spencer_detect)
            pass

        else:
            spencer_detect_temp=DetectedPersons()
            spencer_detect_temp.header.stamp=rospy.get_rostime()
            spencer_detect_temp.header.frame_id="cam_front_color_optical_frame"
            #self.pub_spencer.publish(spencer_detect_temp)

    def detection_callback(self, msg, depth):
        """
        Callback for RGB images: The main logic is applied here

        Args:
        msg (darknet/bounding_box): detections array
        depth (sensor_msgs/image_raw): depth image from camera

        """
        self.lock.acquire()
        if len(self.message)>0:
            self.message=[]
        self.message.append(msg)
        self.message.append(depth)
        self.new_bounding=True
        self.lock.release()


    def get_parameters(self):
        """
        Gets the necessary parameters from parameter server

        Returns:
        (tuple) :
            depth_topic (String): Incoming depth topic name
            face_topic (String): Incoming face bounding box topic name
            output_topic (String): Outgoing depth topic name
            f (Float): Focal Length
            cx (Int): Principle Point Horizontal
            cy (Int): Principle Point Vertical
            id_increment (Int): ID increment for spencer
            id_offset (Int): ID offset for spencer
            output_spencer (String): Output Topic
        """

        depth_topic  = rospy.get_param("~depth_topic")
        face_topic = rospy.get_param('~face_topic')
        output_topic = rospy.get_param('~output_topic')
        f = rospy.get_param('~focal_length')
        cx = rospy.get_param('~cx')
        cy = rospy.get_param('~cy')
        id_increment=rospy.get_param('~detection_id_increment')
        id_offset=rospy.get_param('~detection_id_offset')
        output_spencer=rospy.get_param('~output_topic_spencer')
 
        return (depth_topic, face_topic, output_topic, f, cx, cy,id_increment,id_offset,output_spencer)


def main():
    """ main function
    """
    node = ProjectionNode()

if __name__ == '__main__':
    main()
