#!/usr/bin/env python

'''
camera calibration for distorted images with chess board samples
reads distorted images, calculates the calibration and write undistorted images

'''
import numpy as np
import cv2
import cv2.cv as cv
import rospy
import glob
import os
import sys

class CameraCalibration(object):
    def __init__(self, node_name):
        self.node_name = node_name

        rospy.init_node(node_name)

        rospy.loginfo("Starting node " + str(node_name))

        rospy.on_shutdown(self.cleanup)
        
        # Initialize output folder path to current terminal directory
        self.img_dir = os.path.dirname(os.path.realpath('__file__'))[:-8] + "/include/opencv_camera_calibration/camera_cal_data"

        # Initialize findChessboardCorners and drawChessboardCorners parameters
        self.patternSize_columns = rospy.get_param("~patternSize_columns", 10)
        self.patternSize_rows = rospy.get_param("~patternSize_rows", 8)
        self.square_size = rospy.get_param("~square_size", 0.020)
        
        # Initialize cornerSubPix parameters
        self.winSize_columns = rospy.get_param("~winSize_columns", 5)
        self.winSize_rows = rospy.get_param("~winSize_rows", 5)
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
        img_names = glob(self.img_dir + "/image_*.jpg")
    
        pattern_size = (self.patternSize_columns, self.patternSize_rows)
        pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
        pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
        pattern_points *= self.square_size
    
        obj_points = []
        img_points = []
        i = 0
 
        for fname in img_names:

            print("processing " + img_names[i])
            i += 1
            
            img = cv2.imread(fname, 0)
            
            if img is None:
                print("Failed to load", fname)
                continue
            
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(img, pattern_size, None)
            
            # If found, add object points, image points (after refining them)
            if ret == True:
                corners2 = cv2.cornerSubPix(gray, corners, (self.winSize_columns, self.winSize_rows), (-1, -1), self.criteria)
    
            if not ret:
                print('chessboard not found')
                continue
    
            img_points.append(corners2)
            obj_points.append(pattern_points)
    
            print('ok')
    
        # calculate camera distortion
        rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
        
        tot_error = 0
        
        for i in xrange(len(obj_points)):
            img_points2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coefs)
            error = cv2.norm(img_points[i],img_points2, cv2.NORM_L2)/len(img_points2)
            tot_error += error
        
        output_file = open(self.img_dir + "/camera_calibration.txt", "w+")
        output_file.write("\nRMS:", rms)
        output_file.write("camera matrix:\n", camera_matrix)
        output_file.write("distortion coefficients: ", dist_coefs.ravel())
        output_file.write("mean error: ", tot_error/len(obj_points))

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

def main(args):
    try:
        node_name = "CameraCalibration"
        CameraCalibration(node_name)
        
    except KeyboardInterrupt:
        print "Shutting down CameraCalibration node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
    