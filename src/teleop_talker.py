#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String
from pynput.keyboard import Key, Listener
import os

key_command = {'w': 'Up', 'a': 'Left', 's': 'Down', 'd': 'Right'}

def teleop_talker():
    """ Registra gli input provenienti dalla tastiera e li invia
        attraverso messaggi al topic /teleop_keyboard.
    """
    # Definiamo un nuovo nodo ROS.
    rospy.init_node('teleop_talker', anonymous=True)
    # Registriamo il nuovo nodo come publicatore su un topic.
    pub = rospy.Publisher('teleop_keyboard', String, queue_size=10)
    # Impostiamo la frequenza dei messaggi.
    rate = rospy.Rate(5)

    def on_press(key):
        """ Funzione di callback scatenata alla pressione di un
            qualunque input da tastiera.

            Args:
                key (KeyCode object): oggetto di tipo KeyCode
                identificante l'input da tastiera.
        """
        if hasattr(key, 'char'):
            if key.char in key_command.keys():
                command = key_command[key.char]

                pub.publish(String(command))

                rospy.loginfo('Ho inviato il comando {0}.'.format(command))
            elif key.char == 'q': 
                return False

    def on_release(key):
        """ Funzione di callback scatenata al rilascio di un
            qualunque input da tastiera.

            Args:
                key (KeyCode object): oggetto di tipo KeyCode
                identificante l'input da tastiera.
        """
        if hasattr(key, 'char') and key.char in key_command.keys():
            pub.publish(String('Stop'))

            rospy.loginfo('Ho inviato il comando Stop.')

    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

def print_usage():
    """ Stampa una guida all'utilizzo.
    """
    print("Utilizza i tasti w, a, s, d per la navigazione. Premi q per uscire.\n")
    print((" "*35) + "W" + (" "*10))
    print((" "*25) + "A" + (" "*19) + "S")
    print((" "*35) + "S" + (" "*10))
    print()

if __name__ == "__main__":
    print_usage()
    # Nascondiamo gli input inseriti da tastiera.
    os.system('stty -echo')

    teleop_talker()

    # Rendiamo nuovamente visibili gli input inseriti da tastiera.
    os.system("stty  echo")
