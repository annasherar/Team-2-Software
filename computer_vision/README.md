In progress computer_vision ROS package for the drone. This package deals with accessing and publishing camera images. 
Also deals with computer vision to detect aruco markers and estimate pose with respect to aruco markers.

As of right now, 11/5/23, the two main working files are aruco_node_py.py and camera_node_py.py. These were reworked in python due to issues in C++ that we still have to work on.

The Camera node interfaces with a connected webcam, and publishes image data while the Aruco node subscribes to the webcam topic and process the image data with OpenCV to detect Aruco Markers.
