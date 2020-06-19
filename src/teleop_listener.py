#!/usr/bin/env python3
from std_msgs.msg import String
import serial
import rospy
import time
import os

ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(1)

def _callback(data):
    rospy.loginfo('I received the message {0}.'.format(data.data))

    ser.write(str.encode(data.data))

def teleop_listener():
    ''' 
        Listen to messages from /teleop_keyboard topic and
        forward them to Arduino.
    '''
    # Define a new ROS node.
    rospy.init_node('teleop_listener', anonymous=True)
    # Register the node as subscriber to a specific topic.
    rospy.Subscriber('teleop_keyboard', String, _callback)
    # Use spin() function to keep the code in execution.
    rospy.spin()

if __name__ == '__main__':
    # Hide keyboard inputs.
    os.system('stty -echo')

    teleop_listener()

    # Show keyboard inputs.
    os.system('stty  echo')
