#!/usr/bin/env python
""" 
Description:
    This module subscribes to a topic that outputs an Image message.
    This node converts the image back into a OpenCV type image before
    processing or modifying the image. After processing, the image is
    converted back into an Image message type to be published and 
    available to all nodes under the ros_image_topic.

Node Name:
    ros_image_node 

Publishers:
    ros_image_topic

Subscribers:
    raw_image_topic 

Services:
    None

"""
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class ImageConverter(object):
    """
    Image Converter Class:
    Initializes the publishers and subscribers.
    Contains the callback that is called whenever this is a new image
    published to raw_image_topic.
    """

    def __init__(self):
        self.image_pub = rospy.Publisher(
            "ros_image_topic", Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "raw_image_topic", Image, self.callback)
        self.last_valid_denom = 1  # Used to avoid division by zero while processing

    def callback(self, data):
        """ Callback for when new image is published"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as error:
            print error

        # Example processing - Draws a circle at top left
        """
        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
         cv2.circle(cv_image, (50, 50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        """

        #cv2.imshow('Processing - Before Function', cv_image)
        result = image_processing(self.last_valid_denom, cv_image)

        cv2.imshow('Processed', result)

        cv2.waitKey(3)  # Some delay - REQUIRED

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as error:
            print error


def image_processing(last_valid_denom, frame):
    """Function to apply post - capture image processing"""
    # Example: crop image
    #crop_img = frame[150:480, 0:640] # NOTE: its img[y: y + h, x: x + w]

    # Operations on the frame
    # Convert image to HSV format for processing
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Values in HSV format. use RGB_to_HSV_converter.py to find values
    lower_gray = np.array([0, 0, 110])
    upper_gray = np.array([0, 25, 200])

    # Mask must be applied to HSV image
    mask = cv2.inRange(hsv, lower_gray, upper_gray)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    #reConvert = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)

    # Math - determins center for all points that are left after mask is applied
    moment = cv2.moments(mask, False)
    denom = moment['m00']
    if denom > 0:
        last_valid_denom = denom  # May not actually be set to anything besided initial '1'
    else:
        denom = last_valid_denom
    c_x = int(moment['m10'] / denom)
    c_y = int(moment['m01'] / denom)
    #print moment['m00']

    # Show resulting circle
    cv2.circle(result, (int(c_x), int(c_y)), 10, (0, 0, 255), -1)

    return result


def main():
    """
    Main function
    """
    rospy.init_node('ros_image_node', anonymous=True)
    ImageConverter()
    #i_c = ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
    #main(sys.argv)
