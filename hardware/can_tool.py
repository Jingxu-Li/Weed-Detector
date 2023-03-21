'''
Device-side abtraction of can bus, provide sprayer* method for communication

This file aims to send/recv data from converter
'''

import socket
import threading
import time
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
                id = receive_data[1:5]
                # 15: magic number, mask 1111
                length = int.from_bytes(
                    receive_data[0], byteorder='big', signed=False) & 15
                print(
                    f"Received data {receive_data[5:5 + length]} from id:{id}")

    def send(self, message, server, id_int):
        id = (id_int).to_bytes(4, byteorder='big')
        sub = 0
        length = len(message)
        while length > 8:
            pre = bytes([168]) + id
            self.server_socket.sendto(pre+message[sub:sub + 8], server)
            sub = sub + 8
            length = length - 8

        # 160: magic number, indicates data frame
        pre = bytes([160 + length]) + id
        self.server_socket.sendto(
            pre + message[sub:sub + length] + bytes(8 - length), server)


class DriveFrame(CANConverter):
    id = 0x00
    drive_status = 0
    cut_position = 0
    motor_speed = 0
    car_angle = 0
    message = bytearray()

    def __init__(self, host, port, id):
        super().__init__(host, port)
        self.id = id

    def change_drive_status(self, target):
        self.drive_status = target
        self.compose_frame()

    def change_motor_speed(self, speed):
        self.motor_speed = speed
        self.compose_frame()

    def compose_frame(self):
        self.message[1] = self.drive_status
        self.message[3] = self.cut_position
        # TODO: fulfill complete frame
        self.send()

class NozzleFrame(CANConverter):
    id = 0x00
    open_bits = 0x0000
    message = bytearray()

    def __init__(self, host, port, id):
        super().__init__(host, port)
        self.id = id

    def change_drive_status(self, target):
        self.drive_status = target
        self.compose_frame()

    def change_motor_speed(self, speed):
        self.motor_speed = speed
        self.compose_frame()

    def compose_frame(self):
        self.message[1] = self.open_bits[1]
        # TODO: fulfill complete frame


class Sprayer(NozzleFrame, DriveFrame):

    def __init__(self, host, port):
        super().__init__(host, port)

    def forward(self, time):
        self.change_drive_status(1)
        time.sleep(time)
        self.change_drive_status(3)
        self.change_motor_speed(100)
        time.sleep(time)