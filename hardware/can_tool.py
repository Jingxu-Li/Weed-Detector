'''
Device-side abtraction of can bus, provide sprayer* method for communication

This file aims to send/recv data from converter
'''

import socket
import threading


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
        
    def send_standard_frame(self, message, server, int_id):
        frame_id = (int_id).to_bytes(4, byteorder='big')
        length = 8 # data length

        # 0x20: 00100000
        # Format: FF-RTR-UDP-X-B3-B2-B1-B0
        # FF: 0-Standard frame, 1-Extended frame
        # RTR: 1-Remote frame, 0-Data frame
        # udp: 1-UDP, 0-TCP
        # X: preserve bit
        # B0-B3: data length

        pre = bytes([0x20 + length]) + frame_id
        self.server_socket.sendto(pre + message + bytes(8 - length), server)


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
        self.message = self.message[0:1] + \
            bytes([drive_status])+self.message[2:8]

    def change_header_percent(self, header_percent):
        self.message = self.message[0:3] + \
            bytes([header_percent])+self.message[4:8]

    def change_engine_rpm(self, engine_rpm):
        self.message = self.message[0:4] + \
            (engine_rpm).to_bytes(2, byteorder='big')+self.message[6:8]

    def change_turn_angle(self, turn_angle):
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
        self.message = ((int)(temp*4)).to_bytes(3,
                                                byteorder='big') + self.message[3:8]

    def send_message(self, server):
        print("Sprayer sending message:", self.message)
        super().send_standard_frame(self.message, server, self.id)
