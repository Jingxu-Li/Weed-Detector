'''
main pipeline of sprayer with single camera
'''
from copy import deepcopy
import cv2
# import matplotlib.pyplot as plt
import numpy as np
import time

from hardware.can_tool import sprayerFrame
from detection.detector import Detector
import queue
import threading

width = 640

map_nuzzle = {
    0:[5,6,7],
    1:[2,3,4],
    2:[15,0,1],
    3:[11, 12, 14, 13],
    4:[8,9,10],
    5:[21, 16, 17],
    6:[18,19,20],
}

def get_image(video_port):
    cap = cv2.VideoCapture(video_port, cv2.CAP_V4L2)
    frame = None
    try:
        ret, frame = cap.read()
    except Exception as e:
        print(e)
        return None
    return frame

def set_left(arr, port, value):
    if len(map_nuzzle[port])==4:
        arr[map_nuzzle[port][0]]=value
        arr[map_nuzzle[port][1]]=value
    else:
        arr[map_nuzzle[port][0]]=value
        arr[map_nuzzle[port][1]]=value
    return arr

def set_right(arr, port, value):
    if len(map_nuzzle[port])==4:
        arr[map_nuzzle[port][2]]=value
        arr[map_nuzzle[port][3]]=value
    else:
        arr[map_nuzzle[port][1]]=value
        arr[map_nuzzle[port][2]]=value
    return arr

def spray_without_threads(port, arr, debug=False):
    img = get_image(port)
    if img is None:
        return None
    if not img.any():
        return None
    centers = det.find_weeds_threshold(img, True, port)
    if centers:
        print(f"Found in port:{port}")
        left = False
        right = False
        for center in centers:
            print(center)
            if center[0] < width/2:
                left = True
                arr = set_left(arr,port, 1)
            else:
                right = True
                arr = set_right(arr,port,1)
        if not left:
            arr = set_left(arr,port,0)
        if not right:
            arr = set_right(arr,port,0)
    return arr

q = queue.Queue()

def send_to_sprayer():
    while True:
        try:
            cmd = q.get()
            con.change_spray_head(cmd)
            con.send_message(server)
            print(f"Have sent to server:{cmd}")
            time.sleep(2)
            con.change_spray_head(zero_msg)
            con.send_message(server)
        except:
            continue
        

def prt_cmd(cmd):
    indexes = []
    for i in range(0,len(cmd)):
        if cmd[i]:
            indexes.append(i)
    print(f"indexes:{indexes}")

if __name__ == "__main__":

    debug = False
    if not debug:
        con = sprayerFrame('192.168.1.6', 8001, 0x0202)
    server = ('192.168.1.10', 4001)
    ports = [0,1,2,3,4,5]
    det = Detector()
    zero_msg = np.zeros(22, dtype=int)
    spray_command = np.zeros(22, dtype=int)
    t = threading.Thread(target=send_to_sprayer, daemon=True)
    t.start()
    while True:
        for port in ports:
            spray_command = spray_without_threads(port, spray_command, debug)
            spray_cmd_cp = deepcopy(spray_command)
            q.put(spray_cmd_cp)
        print(spray_command)
