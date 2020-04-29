#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo('Ho ricevuto il comando {0}'.format(data.data))

def teleop_listener():
    """ Ascolta i messaggi provenienti dal topic /teleop_keyboard.
    """
    # Definiamo un nuovo nodo ROS.
    rospy.init_node('teleop_listener', anonymous=True)
    # Registriamo il nodo appena creato come ascoltatore
    # su uno specifico topic.
    rospy.Subscriber('teleop_keyboard', String, callback)
    # Utiliziamo la funzione spin() per mantenere il
    # codice in esecuzione.
    rospy.spin()

if __name__ == "__main__":
    teleop_listener()
