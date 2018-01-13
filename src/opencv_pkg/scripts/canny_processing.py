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
    canny_processing_topic

Subscribers:
    raw_image_topic

Services:
    None

"""
import sys
from collections import deque
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

LINES_QUEUE_LENGTH = 50

left_lines = deque(maxlen=LINES_QUEUE_LENGTH)
right_lines = deque(maxlen=LINES_QUEUE_LENGTH)


class ImageConverter(object):
    """
    Image Converter Class:
    Initializes the publishers and subscribers.
    Contains the callback that is called whenever this is a new image
    published to raw_image_topic.
    """

    def __init__(self):
        self.image_pub = rospy.Publisher(
            "canny_processing_topic", Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "raw_image_topic", Image, self.callback)
        #self.left_lines = deque(maxlen=LINES_QUEUE_LENGTH)
        #self.right_lines = deque(maxlen=LINES_QUEUE_LENGTH)

        #self.last_valid_denom = 1  # Used to avoid division by zero while processing

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
        result = image_processing(cv_image)

        cv2.imshow('Processed', result)

        cv2.waitKey(3)  # Some delay - REQUIRED

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as error:
            print error


def image_processing(frame):
    """Function to apply post - capture image processing"""
    # Example: crop image
    #crop_img = frame[150:480, 0:640] # NOTE: its img[y: y + h, x: x + w]

    # Operations on the frame
    white_yellow_result = select_rgb_white_yellow(frame)
    grayscale_image = convert_gray_scale(frame)
    #grayscale_image = convert_gray_scale(white_yellow_result)
    smooth_image = apply_smoothing(grayscale_image)
    edges_image = detect_edges(smooth_image)  # Apply canny
    ROI_image = select_region(edges_image)
    lines = hough_lines(ROI_image)
    #left_line, right_line = lane_lines(frame, lines)

    """
    full_ROI_image = edges_image
    lines = hough_lines(full_ROI_image)
    left_line, right_line = lane_lines(frame, lines)
    """

    if len(lines[0]) > 0:
        for x1, y1, x2, y2 in lines[0]:
            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
    """
    for x1, y1, x2, y2 in right_line:
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    """
    """
    def mean_line(line, lines):
        if line is not None:
            lines.append(line)

        if len(lines) > 0:
            line = np.mean(lines, axis=0, dtype=np.int32)
            # make sure it's tuples not numpy array for cv2.line to work
            line = tuple(map(tuple, line))
        return line

    left_line = mean_line(left_line,  left_lines)
    right_line = mean_line(right_line, right_lines)

    result = draw_lane_lines(frame, (left_line, right_line))
    """

    result = edges_image

    """
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

    """

    return result


# image is expected be in RGB color space
def select_rgb_white_yellow(image):
    """ Docstring """
    # white color mask
    lower = np.uint8([200, 200, 200])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(image, lower, upper)
    # yellow color mask
    lower = np.uint8([190, 190, 0])
    upper = np.uint8([255, 255, 255])
    yellow_mask = cv2.inRange(image, lower, upper)
    # combine the mask
    mask = cv2.bitwise_or(white_mask, yellow_mask)
    masked = cv2.bitwise_and(image, image, mask=mask)
    return masked


def convert_gray_scale(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


def apply_smoothing(image, kernel_size=9):
    """
    kernel_size must be postivie and odd. Try odd  # 's betwen 3-17.
	Larger kernel_size - - > More processing time and more blur
    """
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)


def detect_edges(image, low_threshold=50, high_threshold=150):
    """ If pixel gradient > high_threshold:
            pixel accepted as edge
        If pixel gradient < low_threshold:
            pixel rejected as edge
        If pixel gradient > low and pixel gradient < high:
            ccepted as edge, only if connected to pixel above high_threshold
    * Need to play aroud with upper/lower thresholds.
      Recommended ratio upper:lower is 2:1 or 3:1
    """
    return cv2.Canny(image, low_threshold, high_threshold)


def filter_region(image, vertices):
    """
    Create the mask using the vertices and apply it to the input image
    """
    mask = np.zeros_like(image)
    # Fills in the 'mask' array of empty zeros with 255 as the value for the
    # points inside the polygon (ROI) specified by the verticies
    if len(mask.shape) == 2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        # in case, the input image has a channel dimension
        cv2.fillPoly(mask, vertices, (255,) * mask.shape[2])
    # AND the original image with the mask, the white polygon area will be 'ANDed'
    # leaving only the white image points within the polygon on the original image.
    # Everything outside the white polygon will become black
    return cv2.bitwise_and(image, mask)


def select_region(image):
    """
    It keeps the region surrounded by the `vertices` (i.e. polygon).  
    Other area is set to 0 (black).
    """
    # First, define the polygon by vertices
    rows, cols = image.shape[:2]
    bottom_left = [cols * 0.1, rows * 0.95]
    top_left = [cols * 0.4, rows * 0.6]
    bottom_right = [cols * 0.9, rows * 0.95]
    top_right = [cols * 0.6, rows * 0.6]
    # the vertices are an array of polygons (i.e array of arrays)
    # and the data type must be integer
    vertices = np.array(
        [[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    return filter_region(image, vertices)


def hough_lines(image):
    """
    `image` should be the output of a Canny transform.
    
    Returns hough lines (not the image with lines)

	rho: Distance resolution of the accumulator in pixels.
	theta: Angle resolution of the accumulator in radians.
	threshold: Accumulator threshold parameter. Only those lines are returned
	           that get enough votes (> threshold).
	minLineLength: Minimum line length. Line segments shorter than that are rejected.
	maxLineGap: Maximum allowed gap between points on the same line to link them.
    """
    return cv2.HoughLinesP(image, rho=1, theta=np.pi / 180, threshold=20,
                           minLineLength=20, maxLineGap=300)


def average_slope_intercept(lines):
    left_lines = []  # (slope, intercept)
    left_weights = []  # (length,)
    right_lines = []  # (slope, intercept)
    right_weights = []  # (length,)
    """
    if lines.all() == None:
        return (1, 1), (-1, 1)
    """

    for line in lines:
        for x1, y1, x2, y2 in line:
            if x2 == x1:
                continue  # ignore a vertical line
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
            length = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
            if slope < 0:  # y is reversed in image
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))

    # add more weight to longer lines
    left_lane = np.dot(left_weights,  left_lines) / \
        np.sum(left_weights) if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / \
        np.sum(right_weights) if len(right_weights) > 0 else None

    return left_lane, right_lane  # (slope, intercept), (slope, intercept)


def make_line_points(y1, y2, line):
    """
    Convert a line represented in slope and intercept into pixel points
    """
    if line is None:
        return None

    slope, intercept = line

    # make sure everything is integer as cv2.line requires it
    slope = slope + 1
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    y1 = int(y1)
    y2 = int(y2)

    return ((x1, y1), (x2, y2))


def lane_lines(image, lines):
    left_lane, right_lane = average_slope_intercept(lines)

    y1 = image.shape[0]  # bottom of the image
    y2 = y1 * 0.6         # slightly lower than the middle

    left_line = make_line_points(y1, y2, left_lane)
    right_line = make_line_points(y1, y2, right_lane)

    return left_line, right_line


def draw_lane_lines(image, lines, color=[255, 0, 0], thickness=20):
    # make a separate image to draw lines and combine with the orignal later
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            # *line - unpack array item into tuple
            #tuple_line = tuple(line)
            # *line line[0] line[1]
            cv2.line(line_image, line[0], line[1], color, thickness)
    # image1 * a + image2 * b + lambda
    # image1 and image2 must be the same shape.
    return cv2.addWeighted(image, 1.0, line_image, 0.95, 0.0)


def main():
    """
    Main function
    """
    rospy.init_node('canny_processing_node', anonymous=True)
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
