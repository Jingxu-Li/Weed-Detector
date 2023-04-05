'''
Device-side abtraction of can bus, provide sprayer* method for communication

This file aims to send/recv data from converter
'''

import socket
import threading
import struct
import time
import random


class CANConverter():
    '''
    CAN Converter here is Ethernet-CAN Converter
    Using udp to send&recv data, bind to local host and port
    '''

    def __init__(self, host, port):
        print(1)
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
                length = int.from_bytes(receive_data[0], byteorder='big', signed=False) & 15
                print(f"Received data {receive_data[5:5 + length]} from id:{id}")

    def send(self, message, server, int_id):
        id = (int_id).to_bytes(4, byteorder='big')
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


class driveFrame(CANConverter):
    id = 0
    drive_status = 0
    header_percent = 0
    engine_rpm = 0
    turn_angle = 0
    message = bytes(8)
    def __init__(self, host, port, id):
        super().__init__(host, port)
        self.id = id
    def change_drive_status(self, drive_status):
        self.message = self.message[0:1]+bytes([drive_status])+self.message[2:8]

    def change_header_percent(self, header_percent):
        self.message = self.message[0:3]+bytes([header_percent])+self.message[4:8]

    def change_engine_rpm(self, engine_rpm):
        self.message = self.message[0:4]+(engine_rpm).to_bytes(2, byteorder='big')+self.message[6:8]

    def change_turn_angle(self,turn_angle):
        self.message = self.message[0:6]+bytes([turn_angle])+self.message[7:8]

    def send(self, server):
        super().send(self.message, server, self.id)

class sprayerFrame(CANConverter):
    message = bytes(8)
    id = 0
    def __init__(self, host, port, id):
        super().__init__(host, port)
        self.id = id
    def change_spray_head(self, msg):
        if len(msg == 22):
            temp = 0
            for item in msg:
                temp = temp*2+item
        self.message = ((int)(temp*4)).to_bytes(3, byteorder='big') + self.message[3:8]
    def send(self, server):
        print(self.message)
        super().send(self.message, server, self.id)
