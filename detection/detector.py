'''
Main algorithm implementation for sprayer detector
'''

import cv2
from imutils import grab_contours
import numpy as np
import matplotlib.pyplot as plt

class Detector():

    def __init__(self):
        pass

    def exg_img(self, img):
        low = 50
        high = 200
        blue = img[:, :, 0].astype(np.float32)
        green = img[:, :, 1].astype(np.float32)
        red = img[:, :, 2].astype(np.float32)
        imgOut = 2 * green - red - blue
        imgOut = np.clip(imgOut, 0, 255)
        imgOut = imgOut.astype('uint8')
        mask = cv2.inRange(imgOut, low, high)
        return mask

    def find_weeds_threshold(self, img, store=False, port=None):
        img_binary = self.exg_img(img)

        # plt.imshow(img_binary)
        # plt.show()

        # remove holes in the surface
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6))
        open_res = cv2.morphologyEx(img_binary, cv2.MORPH_CLOSE, kernel, iterations=5)
        items = cv2.findContours(open_res, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print(contours)
        # cnts = items[0] if len(items) == 2 else items[1]
        counts = grab_contours(items)

        weedCenters = []
        if len(counts) != 0:
            for cnt in counts:
                startX, startY, boxW, boxH = cv2.boundingRect(cnt)
                endX = startX + boxW
                endY = startY + boxH
                centerX = int(startX + (boxW / 2))
                centerY = int(startY + (boxH / 2))
                weedCenters.append([centerX, centerY])
                result = cv2.rectangle(img, (int(startX), int(startY)), (endX, endY), (0, 0, 255), 2)
        # if weedCenters:
        #     plt.imshow(result)
        #     plt.show()
        if store:
            import time
            current_time = time.time()
            cv2.imwrite(f"./log_pics/port_{port}_{current_time}_raw.jpg", img)
            if weedCenters:
                cv2.imwrite(f"./log_pics/port_{port}_{current_time}_read.jpg", result)
            else:
                # print("Not detected")
                cv2.imwrite(f"./log_pics/port_{port}_{current_time}_not.jpg", img_binary)
        return weedCenters

if __name__ == "__main__":
    det = Detector()
    img = cv2.VideoCapture(1)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    centers = det.find_weeds_threshold(img, store=True, port=1)
    print(centers)