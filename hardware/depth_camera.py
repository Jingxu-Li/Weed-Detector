'''
main method to get depth camera info and imu data
Refernece: https://github.com/IntelRealSense/librealsense/issues/6031
'''

from threading import Thread
import pyrealsense2 as rs
import numpy as np
import cv2, sys, time


class VideoStream:
    def __init__(self, resolution=(640, 480), framerate=15):
        self.FOV = 100.4 

        # point conversion
        self.num_sectors = 21 # number of sections
        self.pixel_group = resolution[0] / self.num_sectors
        self.distance_array = [0]*self.num_sectors
        self.depth_index = []

        self.color_image = np.zeros((resolution[0], resolution[1]))
        self.depth_image = self.color_image
        self.camera_stopped = False
        self.resolution = resolution
        self.framerate = framerate
        
        self.imu_pipe = rs.pipeline()
        self.imu_config = rs.config()
        self.imu_config.enable_stream(rs.stream.gyro)
        self.imu_config.enable_stream(rs.stream.accel)
        self.acc = []
        self.gyro = []
        
        self.vid_pipe = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, resolution[0], resolution[1], rs.format.z16, framerate)
        self.config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, framerate)
        
        
    def start_camera(self):
        # start the thread to read frames from the video stream
        self.vid_pipe.start(self.config)
        Thread(target=self.update_cam).start()       

    def start_imu(self):
        # start the thread to read frames from the video stream
        self.imu_pipe.start(self.imu_config)
        Thread(target=self.update_imu).start()

    def update_cam(self):
        try:
            print("got to Aa")
            while True:
                # Wait for a coherent pair of frames: depth and color
                vid_frames = self.vid_pipe.wait_for_frames()
                depth_frame = vid_frames.get_depth_frame()
                color_frame = vid_frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                self.depth_image = np.asanyarray(depth_frame.get_data())
                self.color_image = np.asanyarray(color_frame.get_data())
                
		        # Bin pixels and determine local minima
                depth_index = []
                for idx in range(self.num_sectors):
                    a = int(idx*self.pixel_group)
                    b = int((idx+1)*self.pixel_group)
                    self.depth_image[self.depth_image==0] = 3000
                    depth_index = np.append(depth_index, np.min(self.depth_image[:,a:b]))
                    
                self.depth_index = depth_index 
        except:
            self.vid_pipe.stop()
            print("Error in Vision", sys.exc_info())

        finally:
            self.vid_pipe.stop()


    def update_imu(self):
        try:
            print("got to Ab")
            while True:
                # Wait for a coherent pair of frames: depth and color
                mot_frames = self.imu_pipe.wait_for_frames()
                self.acc = mot_frames[0].as_motion_frame().get_motion_data()
                self.gyro = mot_frames[1].as_motion_frame().get_motion_data()

        except:
            self.imu_pipe.stop()
            print("Error in Vision", sys.exc_info())

        finally:
            self.imu_pipe.stop()



if __name__ == '__main__':

    vs = VideoStream()
    vs.start_imu()
    vs.start_camera()
    
    
    time.sleep(5)
    while True:
        cv2.imshow("image", vs.color_image)
        print("acc, gyro", vs.acc, vs.gyro)
        print ("array", vs.depth_index)
        cv2.waitKey(1)