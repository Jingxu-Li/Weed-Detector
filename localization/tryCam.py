import time
import logging
import numpy as np
import pyrealsense2 as rs


class rs2stream(object):
    """Functions in this class are written with dependency on pyrealsense2 by Intel, including state 
    initialisation, frame acquisition, image resize, and a complete stream init-to-shutdown pipeline.
    """

    def __init__(self,
                 frame_width,
                 frame_height,
                 cam_framerate,
                 acc_framerate,
                 gyro_framerate,
                 enable_rgb=True,
                 enable_imu=True):
        # Switches init
        self.enable_imu = enable_imu
        self.enable_rgb = enable_rgb
        #self.enable_cam = True

        # Frame states init
        self.color_image = None
        self.accel_x = None
        self.accel_y = None
        self.accel_z = None
        self.gyro_x = None
        self.gyro_y = None
        self.gyro_z = None
        self.frames = 0
        self.t_init = time.time()
        self.t_frame = self.t_init

        # Image resize
        #self.resize = (frame_width != Width) or (frame_height != Height)
        #self.frame_width = frame_width
        #self.frame_height = frame_height
        """Set IMU config and start streaming it."""
        self.imu_pipeline = None
        if self.enable_imu:
            self.imu_pipeline = rs.pipeline()
            imu_config = rs.config()
            # Accel and gyro config
            imu_config.enable_stream(rs.stream.accel,
                                     rs.format.motion_xyz32f, acc_framerate)
            imu_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f,
                                     gyro_framerate)
            self.imu_pipeline.start(imu_config)

        """Set RGB camera config and start streaming it."""
        self.cam_pipeline = None
        if self.enable_rgb:
            self.cam_pipeline = rs.pipeline()
            pipe_config = rs.config()
            pipe_config.enable_stream(rs.stream.color, frame_width, frame_height,
                                      rs.format.bgr8, cam_framerate)
            self.cam_pipeline.start(pipe_config)

        #time.sleep(5)

    def stop_pipeline(self):
        """Check the state of IMU/RGB camera and stop them 
        in function shutdown() (defined below)
        """
        if self.imu_pipeline is not None:
            self.imu_pipeline.stop()
            self.imu_pipeline = None
        if self.cam_pipeline is not None:
            self.cam_pipeline.stop()
            self.cam_pipeline = None

    def get_imu(self):
        #t_last = self.t_frame
        #self.t_frame = time.time() - self.t_init
        #self.frames += 1
        try:
            if self.enable_imu:
                imu_frames = self.imu_pipeline.wait_for_frames()
        except Exception as e:
            logging.error(e)
            return
        # Get intertial measurements
        if self.enable_imu:
            accel = imu_frames.first_or_default(
                rs.stream.accel,
                rs.format.motion_xyz32f).as_motion_frame().get_motion_data()
            self.accel_x = accel.x
            self.accel_y = accel.y
            self.accel_z = accel.z
            gyro = imu_frames.first_or_default(
                rs.stream.gyro,
                rs.format.motion_xyz32f).as_motion_frame().get_motion_data()
            self.gyro_x = gyro.x
            self.gyro_y = gyro.y
            self.gyro_z = gyro.z
        # Get inertial measurement timestamp
        #self.t_imu = imu_frames.get_timestamp() if self.enable_imu else None

    def get_rgb(self):
        try:
            if self.enable_rgb:
                image_frames = self.cam_pipeline.wait_for_frames()
        except Exception as e:
            logging.error(e)
            return
        # Get visual measurements (RGB, 8-bit planar array)
        if self.enable_rgb:
            color_frame = image_frames.get_color_frame()
            #self.color_image = np.asanyarray(color_frame.get_data(),
                                             #dtype=np.uint8)
            self.color_image = np.asanyarray(color_frame.get_data())
            #if self.resize:
                #if self.enable_rgb:
                    #self.color_image = cv2.resize(
                        #self.color_image,
                        #(self.frame_width, self.frame_height),
                        #cv2.INTER_NEAREST)
                #else:
                    #None
        # Get visual measurement timestamp
        #self.t_rgb = image_frames.get_timestamp() if self.enable_rgb else None

    def run_imu(self):
        """Get frames, fetch data, and return measurements that include inertial 
        info of each frame. The original code by Ed defines x as pitch, y as yaw, 
        and z as roll for gyroscope. Test actual gyro returns before using this function.
        
        return: 
          acceleration x,y,z: float, angular rate: x,y,z: float
        """
        self.get_imu()
        return self.accel_x, self.accel_y, self.accel_z, self.gyro_x, self.gyro_y, self.gyro_z

    def run_rgb(self):
        """Get frames, fetch RGB images.
        
        return:
          color_image: RGB images in nparray
        """
        self.get_rgb()
        return self.color_image

    def shutdown(self):
        """Camera shutdown by func stop_pipeline(). 
        """
        #self.enable_cam = False
        self.stop_pipeline()
