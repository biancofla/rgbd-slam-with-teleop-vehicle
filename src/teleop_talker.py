#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String
from pynput.keyboard import Key, Listener
import os
import termios
import atexit

key_command = {'w': 'Up', 'a': 'Left', 's': 'Down', 'd': 'Right'}

def teleop_talker():
    # Definiamo un nuovo nodo ROS.
    rospy.init_node('teleop_talker', anonymous=True)
    # Registriamo il nuovo nodo come publicatore su un topic.
    pub = rospy.Publisher('teleop_keyboard', String, queue_size=10)
    # Definiamo un tasso di publicazione dei messaggi.
    rate = rospy.Rate(10)

    def on_press(key):
        if not hasattr(key, 'char'): return False

        if key.char in key_command.keys():
            command = key_command[key.char]

            pub.publish(String(command))

            #rospy.loginfo('Ho inviato il comando {0}.'.format(command))
        elif key.char == 'q': return False

    def on_release(key):
        if not hasattr(key, 'char'): return False

        if key.char in key_command.keys():
            pub.publish(String('Stop'))

            #rospy.loginfo('Ho inviato il comando Stop.')

    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

def enable_echo(enable):
    fd = sys.stdin.fileno()
    new = termios.tcgetattr(fd)

    if enable:
        new[3] |= termios.ECHO
    else:
        new[3] &= not termios.ECHO

    termios.tcsetattr(fd, termios.TCSANOW, new)

if __name__ == "__main__":
    atexit.register(enable_echo, True)
    enable_echo(False)

    teleop_talker()
