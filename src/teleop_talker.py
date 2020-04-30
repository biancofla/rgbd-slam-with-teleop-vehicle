#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String
from pynput.keyboard import Key, Listener
import os

key_command = {'w': 'Up', 'a': 'Left', 's': 'Down', 'd': 'Right'}
current_msg = ''

def on_press(key):
    """ Funzione di callback scatenata alla pressione di un
        qualunque input da tastiera.

        Args:
            key (KeyCode object): oggetto di tipo KeyCode
            identificante l'input da tastiera.
    """
    global current_msg

    if hasattr(key, 'char') and key.char in key_command.keys():
        current_msg = key_command[key.char]
    else:
        current_msg = ''

def on_release(key):
    """ Funzione di callback scatenata al rilascio di un
        qualunque input da tastiera.

        Args:
            key (KeyCode object): oggetto di tipo KeyCode
            identificante l'input da tastiera.
    """
    global current_msg

    if hasattr(key, 'char') and key.char in key_command.keys():
        current_msg = 'Stop'

def teleop_talker():
    """ Registra gli input provenienti dalla tastiera e li invia
        attraverso messaggi al topic /teleop_keyboard.
    """
    # Definiamo un nuovo nodo ROS.
    rospy.init_node('teleop_talker', anonymous=True)
    # Registriamo il nuovo nodo come publicatore su un topic.
    pub = rospy.Publisher('teleop_keyboard', String, queue_size=10)
    # Impostiamo la frequenza dei messaggi inviati al secondo.
    rate = rospy.Rate(10)

    global current_msg

    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # TODO: Le stampe di rospy sembrano non funzionare da bash.
    # Forse è un problema di threading?

    while listener.running and not rospy.is_shutdown():
        if current_msg != '':
            pub.publish(String(current_msg))

            rospy.loginfo('Ho inviato il comando {0}.'.format(current_msg))

            current_msg = ''

        rate.sleep()

def print_usage():
    """ Stampa una guida all'utilizzo.
    """
    print("Utilizza i tasti w, a, s, d per la navigazione. Premi Ctrl+c per uscire.\n")
    print((" "*35) + "W" + (" "*10))
    print((" "*25) + "A" + (" "*19) +  "S")
    print((" "*35) + "S" + (" "*10) + "\n")

if __name__ == "__main__":
    print_usage()
    
    # Nascondiamo gli input inseriti da tastiera.
    os.system('stty -echo')

    teleop_talker()

    # Rendiamo nuovamente visibili gli input inseriti da tastiera.
    os.system("stty  echo")
