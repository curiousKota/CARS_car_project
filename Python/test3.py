from __future__ import print_function, division, absolute_import

import time
import threading

from robust_serial import write_order, Order
from robust_serial.threads import CommandThread, ListenerThread

from robust_serial.utils import open_serial_port, CustomQueue

'''
def reset_command_queue():
    """
    Clear the command queue LOL just realizized new handshake: python starts by sending, arduino starts by listening... eh actully nah

    """
    command_queue.clear()
'''

if __name__ == '__main__':
    try:
        serial_file = open_serial_port(serial_port="/dev/cu.usbmodem1411",baudrate=115200, timeout=None)
    except Exception as e:
        raise e
    is_connected=False
    #print("Order.HEllo evaluates to: ",Order.HELLO)
    while not is_connected:
        write_order(serial_file, Order.HELLO)#basically sends one byte representing the numbe that the broadcast order corresponds to (Hello --> 00000000)
        bytes_array = bytearray(serial_file.read(1))
        if bytes_array[0]==Order.ALREADY_CONNECTED:
            is_connected=True
    print("connected!!")


        
