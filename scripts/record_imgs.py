#!/usr/bin/env python

import numpy as np
import cv2
import cv2.cv as cv
import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

class RecordImgs(object):
    def __init__(self, node_name):
        self.node_name = node_name

        rospy.init_node(node_name)

        rospy.loginfo("Starting node " + str(node_name))

        rospy.on_shutdown(self.cleanup)

        # A number of parameters to determine what gets displayed on the
        # screen. These can be overridden the appropriate launch file
        self.show_text = rospy.get_param("~show_text", True)
        self.feature_size = rospy.get_param("~feature_size", 1)

        # Initialize a number of global variables
        self.frame = None
        self.frame_size = None
        self.frame_width = None
        self.frame_height = None
        self.mask = None
        self.marker_image = None
        self.display_image = None
        self.keystroke = None
        self.cps = 0 # Cycles per second = number of processing loops per second.
        self.cps_values = list()
        self.cps_n_values = 20
        self.resize_window_width = 0
        self.resize_window_height = 0
        self.image_count = 0

        # Initialize output folder path to current terminal directory
        self.output_folder = os.path.dirname(os.path.realpath('__file__'))

        # Initialize findChessboardCorners and drawChessboardCorners parameters
        self.patternSize_columns = rospy.get_param("~patternSize_columns", 10)
        self.patternSize_rows = rospy.get_param("~patternSize_rows", 8)

        # Initialize cornerSubPix parameters
        self.winSize_columns = rospy.get_param("~winSize_columns", 5)
        self.winSize_rows = rospy.get_param("~winSize_rows", 5)
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Create the main display window
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        if self.resize_window_height > 0 and self.resize_window_width > 0:
            cv.ResizeWindow(self.cv_window_name, self.resize_window_width, self.resize_window_height)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the image topic and set the appropriate callback
        # The image topic name can be remapped in the appropriate launch file
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=5)

        # Create an image publisher to output the display_image
        self.image_pub = rospy.Publisher("output_rgb_image", Image, queue_size=5)

    def image_callback(self, data):
        # Store the image header in a global variable
        self.image_header = data.header

        # Time this loop to get cycles per second
        start = rospy.get_time()

        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        frame = self.convert_image(data)

        # Some webcams invert the image
        if self.flip_image:
            frame = cv2.flip(frame, 0)

        # Store the frame width and height in a pair of global variables
        if self.frame_width is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size

        # Create the marker image we will use for display purposes
        if self.marker_image is None:
            self.marker_image = np.zeros_like(frame)

        # Copy the current frame to the global image in case we need it elsewhere
        self.frame = frame.copy()

        # Process the image to detect and track objects or features
        processed_image = self.process_image(frame)

        # Make a global copy
        self.processed_image = processed_image.copy()

        # Merge the processed image and the marker image
        self.display_image = cv2.bitwise_or(self.processed_image, self.marker_image)

        # Handle keyboard events
        self.keystroke = cv.WaitKey(500)

        # Compute the time for this loop and estimate CPS as a running average
        end = rospy.get_time()
        duration = end - start
        fps = int(1.0 / duration)
        self.cps_values.append(fps)
        if len(self.cps_values) > self.cps_n_values:
            self.cps_values.pop(0)
        self.cps = int(sum(self.cps_values) / len(self.cps_values))

        # Display CPS and image resolution if asked to
        if self.show_text:
            font_face = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5

            """ Print cycles per second (CPS) and resolution (RES) at top of the image """
            if self.frame_size[0] >= 640:
                vstart = 25
                voffset = int(50 + self.frame_size[1] / 120.)
            elif self.frame_size[0] == 320:
                vstart = 15
                voffset = int(35 + self.frame_size[1] / 120.)
            else:
                vstart = 10
                voffset = int(20 + self.frame_size[1] / 120.)
            cv2.putText(self.display_image, "CPS: " + str(self.cps), (10, vstart), font_face, font_scale, cv.RGB(255, 255, 0))
            cv2.putText(self.display_image, "RES: " + str(self.frame_size[0]) + "X" + str(self.frame_size[1]), (10, voffset), font_face, font_scale, cv.RGB(255, 255, 0))


        # Process any keyboard commands
        self.keystroke = cv2.waitKey(500)
        if self.keystroke != -1:
            try:
                cc = chr(self.keystroke & 255).lower()
                if cc == 't':
                    self.show_text = not self.show_text
                elif cc == ' ' or 's':
                    cv2.imwrite(self.output_folder + "/image_" + str(self.image_count) +".jpg", self.processed_image)
                    rospy.loginfo("Saving image_" + str(self.image_count) + " to " + self.output_folder)
                    self.image_count += 1
                elif cc == 'q' or '\x1b':
                    # The has press the q key, so exit
                    rospy.signal_shutdown("User hit q key to quit.")
            except:
                pass

        # Publish the display image
        self.publish_display_image(self.display_image)

        # Update the image display
        cv2.imshow(self.node_name, self.display_image)

    def process_image(self, cv_image):
        try:
            gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.patternSize_columns, self.patternSize_rows), None)

            # If found, refine and draw the corners
            if ret == True:

                # Refine corners
                corners2 = cv2.cornerSubPix(gray, corners, (self.winSize_columns, self.winSize_rows), (-1,-1), self.criteria)

                # Draw the corners
                cv2.drawChessboardCorners(self.marker_image, (self.patternSize_columns,self.patternSize_rows), corners2, ret)
        except:
            pass

        return cv_image

    def convert_image(self, ros_image):
            # Use cv_bridge() to convert the ROS image to OpenCV format
            try:
                cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
                return np.array(cv_image, dtype=np.uint8)
            except CvBridgeError, e:
                print e

    def publish_display_image(self, display_image):
            if display_image is not None:
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(display_image, 'bgr8'))

                except CvBridgeError, e:
                    print e
            else:
                return

    def cleanup(self):
            print "Shutting down vision node."
            cv2.destroyAllWindows()

def main(args):
    try:
        node_name = "RecordImgs"
        RecordImgs(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down RecordImgs node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
