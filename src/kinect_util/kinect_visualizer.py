#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import roslib
import rospy
import cv2

class kinect_visualizer:

	def __init__(self):
		# Define a new ROS node.
		rospy.init_node('kinect_visualizer', anonymous=True)
		# Create the CvBridge object.
    	self.bridge = CvBridge()
		# Subscribe to the /rgbd_message topic.
    	self.image_sub = rospy.Subscriber("/rgbd_message", Image, self._callback)

	def _callback(self, data):
		rospy.loginfo('I just receive a RGBD image.')

    	try:
			# Use cv_bridge() to convert the ROS image to OpenCV format.
      		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			# Visualize received image.
      		cv2.imshow("RGBD", cv_image)
      		cv2.waitKey(30)
    	except CvBridgeError as e:
        	rospy.logerr(e)

def main():
  	try:
    	kinect_visualizer()
    	rospy.spin()
	except KeyboardInterrupt:
    	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()