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
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math


avgLeft = (0, 0, 0, 0)
avgRight = (0, 0, 0, 0)


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
    #white_yellow_result = select_rgb_white_yellow(frame)
    #Original_ROI_image = select_region(frame)
    grayscale_image = convert_gray_scale(frame)  # was  white_yello_result
    #color_image = convert_color(grayscale_image)  # was  white_yello_result
    #grayscale_image = convert_gray_scale(white_yellow_result)
    smooth_image = apply_smoothing(grayscale_image)
    edges_image = detect_edges(smooth_image)  # Apply canny
    ROI_image = select_region(edges_image)
    lineMarkedImage = hough_lines(ROI_image)
    #weight_img = weighted_img(lineMarkedImage, frame, a=.8, b=.1, l=0)
    color_image = convert_color(grayscale_image)  # was  white_yello_result

    # Test detected edges by uncommenting this
    #return cv2.cvtColor(ROI_image, cv2.COLOR_GRAY2RGB)
    cv2.imshow('ROI', ROI_image)
    print("Hello")

    #return lineMarkedImage

    # draw output on top of original
    return weighted_img(lineMarkedImage, grayscale_image)

    """
    full_ROI_image = edges_image
    lines = hough_lines(full_ROI_image)
    left_line, right_line = lane_lines(frame, lines)
    """
    """
    if len(lines[0]) > 0:
        for x1, y1, x2, y2 in lines[0]:
            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
    """
    #result = edges_image

    #return result


# image is expected be in RGB color space
def select_rgb_white_yellow(image):
    """ Docstring """
    # white color mask
    lower = np.uint8([200, 200, 200])  # 200 200 200
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


def convert_color(image):
    return cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)


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
    top_left = [cols * 0.2, rows * 0.6]
    bottom_right = [cols * 0.9, rows * 0.95]
    top_right = [cols * 0.8, rows * 0.6]
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
    #lines = cv2.HoughLinesP(image, rho=1, theta=np.pi / 180, threshold=40,
    #                       minLineLength=30, maxLineGap=200)
    lines = cv2.HoughLinesP(image, rho=1, theta=np.pi / 180, threshold=20,
                            minLineLength=20, maxLineGap=300)
    print(lines)
    #line_img = np.zeros(image.shape[0], image.shape[1], 3)
    line_img = np.zeros_like(image)
    draw_lines(line_img, lines)
    return line_img


def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
    """
    NOTE: this is the function you might want to use as a starting point once you want to
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).

    Think about things like separating line segments by their
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of
    the lines and extrapolate to the top and bottom of the lane.

    This function draws `lines` with `color` and `thickness`.
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    global avgLeft
    global avgRight
    # state variables to keep track of most dominant segment
    largestLeftLineSize = 0
    largestRightLineSize = 0
    largestLeftLine = (0, 0, 0, 0)
    largestRightLine = (0, 0, 0, 0)

    if lines is None:
        avgx1, avgy1, avgx2, avgy2 = avgLeft
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [
                 255, 255, 255], 12)  # draw left line
        avgx1, avgy1, avgx2, avgy2 = avgRight
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [
                 255, 255, 255], 12)  # draw right line
        return

    for line in lines:
        for x1, y1, x2, y2 in line:
            size = math.hypot(x2 - x1, y2 - y1)
            slope = ((y2 - y1) / (x2 - x1))
            # Filter slope based on incline and
            # find the most dominent segment based on length
            if (slope > 0.5):  # right
                if (size > largestRightLineSize):
                    largestRightLine = (x1, y1, x2, y2)
                cv2.line(img, (x1, y1), (x2, y2), color,
                         thickness + 10)  # Added +10
            elif (slope < -0.5):  # left
                if (size > largestLeftLineSize):
                    largestLeftLine = (x1, y1, x2, y2)
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)

    # Define an imaginary horizontal line in the center of the screen
    # and at the bottom of the image, to extrapolate determined segment
    imgHeight, imgWidth = (img.shape[0], img.shape[1])
    upLinePoint1 = np.array([0, int(imgHeight - (imgHeight / 3))])
    upLinePoint2 = np.array([int(imgWidth), int(imgHeight - (imgHeight / 3))])
    downLinePoint1 = np.array([0, int(imgHeight)])
    downLinePoint2 = np.array([int(imgWidth), int(imgHeight)])

    # reject lines above horizon
    if (largestLeftLine[1] < upLinePoint1[1]):
        avgx1, avgy1, avgx2, avgy2 = avgLeft
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [
                 255, 255, 255], 12)  # draw left line
        avgx1, avgy1, avgx2, avgy2 = avgRight
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [
                 255, 255, 255], 12)  # draw right line
        return

    if (largestRightLine[1] < upLinePoint1[1]):
        avgx1, avgy1, avgx2, avgy2 = avgLeft
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [
                 255, 255, 255], 12)  # draw left line
        avgx1, avgy1, avgx2, avgy2 = avgRight
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [
                 255, 255, 255], 12)  # draw right line
        return

    # Find the intersection of dominant lane with an imaginary horizontal line
    # in the middle of the image and at the bottom of the image.
    p3 = np.array([largestLeftLine[0], largestLeftLine[1]])
    p4 = np.array([largestLeftLine[2], largestLeftLine[3]])

    upLeftPoint = seg_intersect(upLinePoint1, upLinePoint2, p3, p4)
    downLeftPoint = seg_intersect(downLinePoint1, downLinePoint2, p3, p4)
    if (math.isnan(upLeftPoint[0]) or math.isnan(downLeftPoint[0])):  # If invalid
        avgx1, avgy1, avgx2, avgy2 = avgLeft
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [
                 255, 255, 255], 12)  # draw left line
        avgx1, avgy1, avgx2, avgy2 = avgRight
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [
                 255, 255, 255], 12)  # draw right line
        return
    cv2.line(img, (int(upLeftPoint[0]), int(upLeftPoint[1])), (int(      # <------------------
        downLeftPoint[0]), int(downLeftPoint[1])), [0, 0, 255], 18)  # draw left line

    # Calculate the average position of detected left lane over multiple video frames and draw
    #global avgLeft
    avgx1, avgy1, avgx2, avgy2 = avgLeft
    avgLeft = (movingAverage(avgx1, upLeftPoint[0]), movingAverage(avgy1, upLeftPoint[1]), movingAverage(
        avgx2, downLeftPoint[0]), movingAverage(avgy2, downLeftPoint[1]))
    avgx1, avgy1, avgx2, avgy2 = avgLeft
    cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)),
             [255, 255, 255], 12)  # draw left line

    # Find the intersection of dominant lane with an imaginary horizontal line
    # in the middle of the image and at the bottom of the image.
    p5 = np.array([largestRightLine[0], largestRightLine[1]])
    p6 = np.array([largestRightLine[2], largestRightLine[3]])

    upRightPoint = seg_intersect(upLinePoint1, upLinePoint2, p5, p6)
    downRightPoint = seg_intersect(downLinePoint1, downLinePoint2, p5, p6)
    if (math.isnan(upRightPoint[0]) or math.isnan(downRightPoint[0])):
        avgx1, avgy1, avgx2, avgy2 = avgLeft
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [
                 255, 255, 255], 12)  # draw left line
        avgx1, avgy1, avgx2, avgy2 = avgRight
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [
                 255, 255, 255], 12)  # draw right line
        return
    cv2.line(img, (int(upRightPoint[0]), int(upRightPoint[1])), (int(  # <----------
        downRightPoint[0]), int(downRightPoint[1])), [0, 0, 255], 8)  # draw left line

    # Calculate the average position of detected right lane over multiple video frames and draw
    #global avgRight
    avgx1, avgy1, avgx2, avgy2 = avgRight
    avgRight = (movingAverage(avgx1, upRightPoint[0]), movingAverage(avgy1, upRightPoint[1]), movingAverage(
        avgx2, downRightPoint[0]), movingAverage(avgy2, downRightPoint[1]))
    avgx1, avgy1, avgx2, avgy2 = avgRight
    cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)),
             [255, 255, 255], 12)  # draw left line


def perp(a):
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return


def seg_intersect(a1, a2, b1, b2):
    da = a2 - a1
    db = b2 - b1
    dp = a1 - b1
    dap = perp(da)
    denom = np.dot(dap, db)
    num = np.dot(dap, dp)
    return (num / denom.astype(float)) * db + b1


def movingAverage(avg, new_sample, N=20):
    if (avg == 0):
        return new_sample
    avg -= avg / N
    avg += new_sample / N
    return avg


def weighted_img(img, initial_img, a=0.8, b=1., l=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.

    `initial_img` should be the image before any processing.

    The result image is computed as follows:

    initial_img * a + img * b + l
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, a, img, b, l)


def main():
    """
    Main function
    """
    rospy.init_node('canny_processing_node', anonymous=True)
    ImageConverter()
    #avgLeft = (0, 0, 0, 0)
    #avgRight = (0, 0, 0, 0)
    #i_c = ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
    #main(sys.argv)
