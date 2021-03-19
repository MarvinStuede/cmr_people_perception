#!/usr/bin/env python
"""
A ROS node to get 3D values of bounding boxes returned by face_recognizer node.

This node gets the face bounding boxes and gets the real world coordinates of
them by using depth values. It simply gets the x and y values of center point
and gets the median value of face depth values as z value of face.

Author:
    Cagatay Odabasi -- cagatay.odabasi@ipa.fraunhofer.de
"""

import rospy

import message_filters

import numpy as np

from cv_bridge import CvBridge

from sensor_msgs.msg import Image

from cob_perception_msgs.msg import DetectionArray

from spencer_tracking_msgs.msg import DetectedPersons , DetectedPerson


class ProjectionNode(object):
    """Get 3D values of bounding boxes returned by face_recognizer node.

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

        self._bridge = CvBridge()

        (depth_topic, face_topic, output_topic, f, cx, cy,id_increment,id_offset,output_spencer) = \
            self.get_parameters()

         # Subscribe to the face positions
        sub_obj = message_filters.Subscriber(face_topic,\
            DetectionArray)

        sub_depth = message_filters.Subscriber(depth_topic,\
            Image)

        # Advertise the result of Face Depths
        self.pub = rospy.Publisher(output_topic, \
            DetectionArray, queue_size=1)
            
        self.pub_spencer=rospy.Publisher(output_spencer, DetectedPersons,queue_size=1)
        # Create the message filter
        ts = message_filters.ApproximateTimeSynchronizer(\
            [sub_obj, sub_depth], \
            2, \
            0.9)

        ts.registerCallback(self.detection_callback)

        self.f = f
        self.cx = cx
        self.cy = cy

        self.detection_id_increment=id_increment
        self.detection_id_offset=id_offset
        self.current_detection_id=self.detection_id_offset
    
        # spin
        rospy.spin()


    def shutdown(self):
        """
        Shuts down the node
        """
        rospy.signal_shutdown("See ya!")

    def detection_callback(self, msg, depth):
        """
        Callback for RGB images: The main logic is applied here

        Args:
        msg (cob_perception_msgs/DetectionArray): detections array
        depth (sensor_msgs/PointCloud2): depth image from camera

        """

        cv_depth = self._bridge.imgmsg_to_cv2(depth, "passthrough")

        # get the number of detections
        no_of_detections = len(msg.detections)
        spencer_detected_persons = DetectedPersons()
        spencer_detected_persons.header=msg.header
        # Check if there is a detection
        if no_of_detections > 0:
            for i, detection in enumerate(msg.detections):
                
                x =  detection.mask.roi.x
                y = detection.mask.roi.y
                width =  detection.mask.roi.width
                height = detection.mask.roi.height
                scale=0.3
                height_delta=(height-height*scale)/2
                y=y=height_delta
                
                width_delta =(width-width*scale)/2

                cv_depth_bounding_box = cv_depth[int(y+height_delta):int(y+height_delta+height*scale),int(x+width_delta):int(x+width_delta+width*scale)]

                try:

                    depth_mean = np.nanmedian(\
                       cv_depth_bounding_box[np.nonzero(cv_depth_bounding_box)])
                   
                    real_x = (x + width/2-self.cx)*(depth_mean)/self.f

                    real_y = (y + height/2-self.cy)*(depth_mean)/self.f

                    msg.detections[i].pose.pose.position.x = real_x
                    msg.detections[i].pose.pose.position.y = real_y
                    msg.detections[i].pose.pose.position.z = depth_mean
                    if(msg.detections[i].label == 'person'):
                        spencer_detected_person=DetectedPerson()
                        spencer_detected_person.modality=spencer_detected_person.MODALITY_GENERIC_RGBD
                        spencer_detected_person.confidence=1
                        spencer_detected_person.pose.pose=msg.detections[i].pose.pose
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

                except Exception as e:
                    print e

        self.pub.publish(msg)
        self.pub_spencer.publish(spencer_detected_persons)

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
