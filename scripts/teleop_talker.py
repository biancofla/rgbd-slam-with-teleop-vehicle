#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String
from pynput.keyboard import Key, Listener

available_cmd = ['w', 'a', 's', 'd']
current_msg = ''

def _on_press(key):
    global current_msg

    if hasattr(key, 'char') and key.char in available_cmd:
        current_msg = key.char
    else:
        current_msg = ''

def _on_release(key):
    global current_msg

    if hasattr(key, 'char') and key.char in available_cmd:
        current_msg = 'x'

def teleop_talker():
    """ 
        Register keyboard events and send them, as String
        messages, to /teleop_keyboard topic.
    """
    # Define a new ROS node.
    rospy.init_node('teleop_talker', anonymous=True)
    # Register the node as publisher to a specific topic.
    pub = rospy.Publisher('teleop_keyboard', String, queue_size=10)
    # Set sent messages frequency per second.
    rate = rospy.Rate(10)

    global current_msg

    listener = Listener(on_press=_on_press, on_release=_on_release)
    listener.start()

    while not rospy.is_shutdown():
        if current_msg != '':
            pub.publish(String(current_msg))

            rospy.loginfo('I sent the message {0}.'.format(current_msg))

            current_msg = ''

        rate.sleep()

def print_usage():
    """ 
        Print an helper guide.
    """
    print("Use w, a, s, d to move the robot, release to stop it.", \
          "Press Ctrl+c to exit.\n")
    print((" "*35) + "w" + (" "*10))
    print((" "*25) + "a" + (" "*19) +  "d")
    print((" "*35) + "s" + (" "*10) + "\n")

if __name__ == "__main__":
    print_usage()
    
    # Hide keyboard inputs.
    os.system('stty -echo')

    teleop_talker()

    # Show keyboard inputs.
    os.system("stty  echo")
