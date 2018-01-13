#!/usr/bin/env python
""" 
Description:
    This module captures a raw image using the OpenCV library
    and converts it into the ROS Image type message. The Image is
    then published to a topic called "raw_image_topic" for all
    nodes to utilize.

Node Name:
    raw_camera_stream

Publishers:
    raw_image_topic

Subscribers:
    None

Services:
    None

"""
import sys
import signal
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
    """
    Main function of raw_camera_stream.
    Continuously captures images, frame-by-frame and publishes to PUB_CV topic
    """
    while True:
        # Capture frame-by-frame
        ret, frame = CAPTURE.read()

        if ret == 0:  # If image reading fails, try again
            continue

        # Example: crop image
        # crop_img = frame[150:480, 0:640] # NOTE: its img[y: y + h, x: x + w]

        # Display the resulting frames
        cv2.imshow('Original', frame)

        # cv2.wait acts as a minimum delay and a way to check if window should be closed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # Publish raw image to raw_image_topic
        PUB_CV.publish(CV_BRIDGE.cv2_to_imgmsg(frame, "bgr8"))

    # Cleanup if program called to quit
    CAPTURE.release()
    cv2.destroyAllWindows()


def signal_handler(signal_caught, frame):
    """ Handles SIGNINT signal to quit program"""
    print "Exiting..."
    CAPTURE.release()
    cv2.destroyAllWindows()
    sys.exit(0)


if __name__ == '__main__':
    #---------Initialize signal handler-----------------------------------------
    signal.signal(signal.SIGINT, signal_handler)

    #----------Initialize ROS things--------------------------------------------
    rospy.init_node('raw_camera_stream')  # Create openCV processing node
    PUB_CV = rospy.Publisher("raw_image_topic", Image,
                             queue_size=10)  # Add queue_size
    # Get camera source parameter, defaults to 1
    CAMERA_SOURCE = rospy.get_param('camera_source', 1)
    CAPTURE = cv2.VideoCapture(CAMERA_SOURCE)  # Launch camera source
    print ""
    print "---------------------CHANGE CAMERA SOURCE-----------------------"
    print "Camera souce can be changed by specifing camera_source parameter"
    print "Example for first camera: "
    print "roslaunch .. .. camera_source:=0"
    print "----------------------------------------------------------------"
    print ""

    # Bridge allows openCV images to be converted into ROS image messages
    CV_BRIDGE = CvBridge()
    main()  # Main is always running
    rospy.spin()  # To make sure python doesn't exit

    # When everything done, release the capture
    CAPTURE.release()
    cv2.destroyAllWindows()
