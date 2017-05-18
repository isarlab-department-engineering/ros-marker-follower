this node performs the following of an aruco marker. 

to run it usb_cam and ros_markers_detection are needed. you must also include on your workspace the motor_hat node, because ros_marker_follower just finds the proper values for
the 2 wheels with the information that it gets from ros_marker_detection. motor_hat is going to get these values and make the duckiebot move accordingly with them.

then just clone this node in your workspace and run it through rosrun typing:

rosrun aruco_detection aruco_detection distanceInCmToKeepFromMarker