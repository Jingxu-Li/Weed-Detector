'''
main pipeline of sprayer with single camera
'''

import cv2
import matplotlib.pyplot as plt
import numpy as np
import sys
import time

from hardware.can_tool import sprayerFrame

debug = False

def initial_cap():
    cap = cv2.VideoCapture(1 + cv2.CAP_DSHOW)
    flag = cap.isOpened()
    if not flag:
        print("Camera is not opened")
    else:
        print("Camera opened")
        return cap

def get_image(cap, debug_mode):
    frame = None
    if debug_mode:
        frame = cv2.imread('./detection/example.jpg')
    else:
        try:
            ret, frame = cap.read()
        except Exception as e:
            print(e)
    return frame

if __name__ == "__main__":
    con1 = sprayerFrame('192.168.1.6', 8001, 0x0202)
    b = np.zeros(22, dtype=int)
    a = np.zeros(22, dtype=int)
    b[4:8]=1
    server = ('192.168.1.10', 4001)

    if not debug:
        cap = initial_cap()
    else:
        cap = None

    img = get_image(cap, debug)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    plt.imshow(np.asarray(img))
    color_low = np.array([190, 190, 190])
    color_high = np.array([255, 255, 255])
    mask = cv2.inRange(img, color_low, color_high)
    # plt.imshow(np.asarray(mask))

    # remove holes in the surface
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    open_res = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    # plt.imshow(np.asarray(open_res))
    plt.show()
    contours, _ = cv2.findContours(open_res, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area = []

    if len(contours) != 0:
        for i in range(len(contours)):
            area.append(cv2.contourArea(contours[i]))
        max_idx = np.argmax(area)
        con1.change_spray_head(b)
        con1.send_message(server)
        time.sleep(5)
        con1.change_spray_head(a)
        con1.send_message(server)
    else:
        sys.exit(0)

    min_box = cv2.minAreaRect(contours[max_idx])
    print(min_box)
    pts = np.int0(cv2.boxPoints(min_box))
    cv2.drawContours(open_res, [pts], 0, (255, 0, 0), 3)
    cv2.imwrite("result.jpg", open_res)