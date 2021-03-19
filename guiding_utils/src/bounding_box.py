#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.
This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import message_filters
import cv2
# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False
class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
            CompressedImage,queue_size=1)
        # self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/cam_front/color/image_raw/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /camera/image/compressed"

         # Subscribe to the face positions
        sub_obj = message_filters.Subscriber("darknet_ros/bounding_boxes,\
            BoundingBoxes)
        
        sub_color = message_filters.Subscriber("/cam_back_rotated/color/image_raw",Image)
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


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)
        
        #self.subscriber.unregister()

def main(args):
    '''Initializes and cleanup ros node'''
    
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
