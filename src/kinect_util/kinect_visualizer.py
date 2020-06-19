#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import roslib
import rospy
import cv2
import sys

class kinect_visualizer:

  def __init__(self):
    rospy.init_node('kinect_visualizer', anonymous=True)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rgbd_message", Image, self._callback)

  def _callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      
      cv2.imshow("rgbd", cv_image)
      cv2.waitKey(30)
      
      rospy.loginfo('I just retrive a RGBD image.')
    except CvBridgeError as e:
        rospy.logerr(e)

def main(args):
  try:
    kinect_visualizer()
    rospy.spin()
  except KeyboardInterrupt:
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)