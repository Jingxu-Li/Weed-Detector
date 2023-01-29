'''
This file aims to load ControlCAN.dll to send/receive data from serial
'''
from ctypes import cdll, c_ushort, c_char, c_ubyte, c_uint, POINTER
from _ctypes import Structure, byref
import logging
import threading
from concurrent.futures import ThreadPoolExecutor

class PyVCIBOARD_INFO(Structure):
    _fields_ = [
        ("hw_Version", c_ushort),
        ("fw_Version", c_ushort),
        ("dr_Version", c_ushort),
        ("in_Version", c_ushort),
        ("irq_Num", c_ushort),
        ("can_Num", c_ubyte),
        ("str_Serial_Num", c_char*20),
        ("str_hw_Type", c_char*40),
        ("Reserved", c_ushort*4)
    ]


class PyVCI_CAN_OBJ(Structure):
    '''
    data type of data frame
    '''
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("TimeFlag", c_ubyte),
        ("SendType", c_ubyte),
        ("RemoteFlag", c_ubyte),
        ("ExternFlag", c_ubyte),
        ("DataLen", c_ubyte),
        ("Data", c_ubyte*8),
        ("Reserved", c_ubyte*3)
    ]


class PyVCI_CAN_STATUS(Structure):
    _fields_ = [
        ("c_uint", c_ubyte),
        ("regMode", c_ubyte),
        ("regStatus", c_ubyte),
        ("regALCapture", c_ubyte),
        ("regECCapture", c_ubyte),
        ("regEWLimit", c_ubyte),
        ("regRECounter", c_ubyte),
        ("regTECounter", c_ubyte),
        ("Reserved", c_uint)
    ]


class PyVCI_ERR_INFO(Structure):
    _fields_ = [
        ("ErrCode", c_uint),
        ("Passive_ErrData", c_ubyte*3),
        ("ArLost_ErrData", c_ubyte)
    ]


class PyVCI_INIT_CONFIG(Structure):
    _fields_ = [
        ("AccCode", c_uint),
        ("AccMask", c_uint),
        ("Reserved", c_uint),
        ("Filter", c_ubyte),
        ("Timing0", c_ubyte),
        ("Timing1", c_ubyte),
        ("Mode", c_ubyte)
    ]


class CanConverter():
    def __init__(self, serial_port):
        self.libUSBCAN = cdll.LoadLibrary(("/lib/libusbcan.so"))
        self.VCI_USBCAN1 = 3  # USB to CAN
        openDevice = self.libUSBCAN.VCI_OpenDevice
        openDevice.argtype = [c_uint, c_uint, c_uint]
        openDevice.restype = c_uint

        # inital device
        if(openDevice(self.VCI_USBCAN1, 0, 0) == 1):
            logging.debug("open device success")
        else:
            logging.debug("open device failed")

        # Py_BOARD_INFO = PyVCIBOARD_INFO()
        # readBoardInfo = self.libUSBCAN.VCI_ReadBoardInfo
        # readBoardInfo.argtype = [c_uint, c_uint, POINTER(PyVCIBOARD_INFO)]
        # readBoardInfo.restype = c_uint

        initCAN = self.libUSBCAN.VCI_InitCAN
        initCAN.argtype = [c_uint, c_uint, c_uint, POINTER(PyVCI_INIT_CONFIG)]
        initCAN.restype = c_uint

        config = PyVCI_INIT_CONFIG()
        config.AccCode = 0x00000000
        config.AccMask = 0x00000000
        config.Filter = 8
        config.Timing0 = 0x00         # Reference to CAN API, here define 1000Kbps
        config.Timing1 = 0x14
        config.Mode = 0

        if(initCAN(self.VCI_USBCAN1, 0, 0, byref(config)) != 1):
            logging.debug("Init CAN1 error")
            self.libUSBCAN.VCI_CloseDevice(self.VCI_USBCAN1, 0)
            return
        else:
            logging.debug("VCI_InitCAN success!")

        if(self.libUSBCAN.VCI_StartCAN(self.VCI_USBCAN1, 0, 0) != 1):
            logging.debug("Start CAN1 error")
            self.libUSBCAN.VCI_CloseDevice(self.VCI_USBCAN1, 0)
            return
        else:
            logging.debug("VCI_StartCAN success!")

        # initial threads for sending & receiving
        try:
            self.receive_thread = threading.Thread(target=self.receive)
            self.send_thread = threading.Thread(target=self.send)
        except Exception as e:
            logging.debug("VCI_StartCAN success!")
        self.pool = ThreadPoolExecutor(max_workers=10)

    def receive(self):
        '''
        Directly reveive a data frame regardless of data and information
        '''
        reclen = 0
        rec = (PyVCI_CAN_OBJ*300)()

    def send(self):
        '''
        Directly send a data frame regardless of data and information
        '''
        pass


class sprayerConverter(CanConverter):
    def __init__(self, serial_port):
        super().__init__(serial_port)

    def send_spray(field):
        pass

    def receive_amount():
        pass