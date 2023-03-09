'''
Device-side abtraction of can bus, provide sprayer* method for communication

This file aims to send/recv data from converter
'''

import socket
import threading
import random


class CANConverter():
    '''
    CAN Converter here is Ethernet-CAN Converter
    Using udp to send&recv data, bind to local host and port
    '''

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        address = (host, port)
        self.server_socket.bind(address)
        self.receive_thread = threading.Thread(target=self.receive)

    def receive(self):
        while True:
            receive_data, client = self.server_socket.recvfrom(1024)
            if receive_data:
                print(f"Received data {receive_data} from {client}")

    def send(self, message, server):
        sub = 0
        length = len(message)
        id = bytes([random.randint(0,255), random.randint(0,255), random.randint(0,255), random.randint(0,255)])
        while length > 8 :
            pre = bytes([168]) + id
            self.server_socket.sendto(pre+message[sub:sub + 8], server)
            sub = sub + 8
            length = length - 8
        pre = bytes([160 + length]) + id
        self.server_socket.sendto(pre + message[sub:sub + length] + bytes(8 - length), server)

