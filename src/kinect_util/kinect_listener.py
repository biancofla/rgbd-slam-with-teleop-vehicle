#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters
import numpy as np
import rospy
import cv2

class kinect_listener():

    def __init__(self):
        # Define a new ROS node.
        rospy.init_node('kinect_listener', anonymous=True)
        # Create the CvBridge object.
        self.bridge = CvBridge()
        # Subscribe to the camera image and depth topics.
        self.image_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/camera/depth_registered/image_raw', Image)
        # Set an ApproximateTimeSynchronizer object to synchronize messages received.
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], \
            queue_size=10, slop=0.5)
        self.ts.registerCallback(self._callback)
        # Register the node as publisher to a specific topic.
        self.rgbd_pub = rospy.Publisher('/rgbd_message', Image, queue_size=1)	

    def _callback(self, data_rgb, data_d):
        try:
            # Use cv_bridge() to convert the ROS image to OpenCV format.
            rgb_image = self.bridge.imgmsg_to_cv2(data_rgb, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(data_d, '32FC1')
            # Convert the depth image to a Numpy array.
            depth_array = np.array(depth_image, dtype=np.float32)
            # Normalize the depth image pixel values in range [0; 1].
            cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
            # Cast depth_array to unsigned int 8-bit.
            depth_8 = (depth_array * 255).round().astype(np.uint8)
            # Create a numpy array of the same dimension of rgb_image.
            depth_image = np.zeros_like(rgb_image)
            # Assign depth_8 to the three layers of depth_image.
            depth_image[:, :, 0] = depth_8
            depth_image[:, :, 1] = depth_8
            depth_image[:, :, 2] = depth_8
            # Concatenate rgb_image and depth_image.
            rgbd_image = np.concatenate((rgb_image, depth_image), axis=1)

            rgbd_message = self.bridge.cv2_to_imgmsg(rgbd_image, 'bgr8')
            self.rgbd_pub.publish(rgbd_message)

            rospy.loginfo('I just send a RGBD image.')
        except CvBridgeError as e:
            rospy.logerr(e)
    
def main():       
    try:
        kinect_listener()
        # Use spin() function to keep the code in execution.
        rospy.spin()
    except KeyboardInterrupt:
        cv2.DestroyAllWindows()

if __name__ == '__main__':
    main()