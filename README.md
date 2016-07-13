# OpenCV Camera Calibration

## Maintainer

- [Chris Mobley](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<cmobley7@vt.edu>>, [Computational Multiphysics Systems Laboratory](http://www.me.vt.edu/cms/), Virginia Tech

*Note:* ***Please use the GitHub issues*** *for questions and problems regarding the opencv_camera_calibration package and its components.* ***Do not write emails.***

## Description

This tool uses OpenCV to calibrate a camera via a ROS Image stream. It uses chess or circle boards.

## Dependencies

- ROS Indigo
- OpenCV

*for the ROS packages look at the package.xml*

## Key bindings

Windows:
- `ESC`, `q`: Quit
- `SPACE`, `s`: Save the current image for calibration
- `t`: Toggle text on screen

Terminal:
- `CRTL`+`c`: Quit

## Calibration patterns

Checkboard Patterns are available at OpenCV:
- [Chessboard pattern](http://docs.opencv.org/2.4.2/_downloads/pattern.png)
- [Asymmetric circle grid](http://docs.opencv.org/2.4.2/_downloads/acircles_pattern.png)

## Calibrating the Kinect One

*Recommended preparation:*
- Print your calibration pattern and glue it to a flat object. It is very important that the calibration pattern is very flat.
- Get two tripods, one for holding the calibration pattern, and another one for holding the camera sensor. Ideally, the tripod for the camera will have a ball head, to allow you to move it easily and lock it in place before you take an image. It is very important that the sensor is stable (and the image is clear and not blurred) before you take an image.
- When recording images, start the recording program, then press spacebar to record each image. The calibration pattern should be detected (indicated by color lines overlayed on the calibration pattern), and the image should be clear and stable.
- It is recommended to take images that show the calibration pattern in all areas of the image, and with different orientations of the pattern (tilting the pattern relative to the plane of the image), and at least two distances. So you can easily reach 100 images per calibration set.
- We normally start at a short distance, where the calibration pattern covers most of the image, there we take several pictures tilting the calibration pattern vertically, then horizontally. Imagine a ray coming out of the camera sensor, this makes sure that you have images where the calibration pattern is not perpendicular to that ray.
- Then we move the calibration pattern further away, and for different orientations (tilting) of the pattern, we take many images so that we calibration pattern is present around most of the camera image. For example, at first the calibration pattern is on the left upper corner. Then on the next image on the upper middle, then on the upper right corner. Then some images where the calibration pattern is in the middle vertically, etc...

*Detailed steps:*


0. Change `record_images.launch` and `camera_calibration.launch` files to match the topic of the camera and parameters of the checkerboard choosen
1. If you haven't already, start the camera
2. Record images for the camera: `roslaunch opencv_camera_calibration record_imgs.launch`
3. Calibrate the intrinsics and extrinsics parameters: `roslaunch opencv_camera_calibration camera_calibration.launch`
4. Copy the data into the chosen camera .yaml file
5. Restart the camera
