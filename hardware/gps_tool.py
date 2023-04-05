import io

import pynmea2
import serial


class GNSS:

    def __init__(self, port, baud_rate):
        # Example: port = "COM5" and baud_rate = 115200
        self.ser = serial.Serial(port, baud_rate, timeout=5.0)
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))
        self.lat = 0
        self.lon = 0

    def receive(self):
        while 1:
            try:
                # Update gnss information
                line = self.sio.readline()
                msg = pynmea2.parse(line)
                self.lat = msg.latitude
                self.lon = msg.longitude
                # print(repr(msg))
            except serial.SerialException as e:
                print('Device error: {}'.format(e))
                break
            except pynmea2.ParseError as e:
                print('Parse error: {}'.format(e))
                continue
