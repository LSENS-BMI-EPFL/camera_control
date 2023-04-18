"""
Camera Control
Copyright M. Mathis Lab
Written by  Gary Kane - https://github.com/gkane26
post-doctoral fellow @ the Adaptive Motor Control Lab
https://github.com/AdaptiveMotorControlLab

camera class for imaging source cameras - helps load correct settings
"""

import tisgrabber as ic
import numpy as np
import os
import json
import cv2
import ctypes as C
import time


path = os.path.dirname(os.path.realpath(__file__))
dets_file = os.path.normpath(path + '/camera_details.json')
cam_details = json.load(open(dets_file, 'r'))

class CallbackUserdata(C.Structure):
    """ User data passed to the callback function. """
    def __init__(self, camera_index):
        self.camera_index = camera_index
        self.trigger_times = []
        self.callback_times = []
        self.framenumbers = []
        self.saved_times = []
        self.image_description = None
        self.vid_out = None

def frame_callback(hGrabber, pBuffer, framenumber, pData):
    """
    Callback function for each incoming frame.
    It is recommended to perform image processing in the callback, because each hGrabber object has
    its own thread and if many cameras are used, the callback run in own threads. That saves CPU load.

    :param: hGrabber: This is the real pointer to the grabber object. Do not use.
    :param: pBuffer : Pointer to the first pixel's first byte
    :param: framenumber : Number of the frame since the stream started
    :param: pData : Pointer to additional user data structure
    """
    # Get image description values
    Width = pData.image_description[0]
    Height = pData.image_description[1]
    BitsPerPixel = pData.image_description[2]
    colorformat = pData.image_description[3]
#
    # Calculate the buffer size, write frame to video object
    bpp = int(BitsPerPixel / 8.0)
    buffer_size = Width * Height * bpp

    # Log callbacks
    pData.callback_times.append(time.time())
    if buffer_size > 0:
        image = C.cast(pBuffer,
                            C.POINTER(
                                C.c_ubyte * buffer_size))
#
        frame = np.ndarray(buffer=image.contents,
                                 dtype=np.uint8, #Y800
                                 shape=(Height,
                                        Width,
                                        bpp))

        pData.vid_out.write(cv2.flip(frame,0))
        pData.saved_times.append(time.time())
        pData.framenumbers.append(framenumber)
    return


FRAMEREADYCALLBACK = C.CFUNCTYPE(C.c_void_p, C.c_int, C.POINTER(C.c_ubyte), C.c_ulong, C.py_object)  # original
callback_function_ptr = FRAMEREADYCALLBACK(frame_callback)

class ICCam(object):

    def __init__(self, cam_num=0, rotate=None, crop=None, exposure=None):
        '''
        Params
        ------
        cam_num = int; camera number (int)
            default = 0
        crop = dict; contains ints named top, left, height, width for cropping
            default = None, uses default parameters specific to camera
        '''

        self.cam_num = cam_num
        self.rotate = rotate if rotate is not None else cam_details[str(self.cam_num)]['rotate']
        self.crop = crop if crop is not None else cam_details[str(self.cam_num)]['crop']
        self.exposure = exposure if exposure is not None else cam_details[str(self.cam_num)]['exposure']

        self.user_data = CallbackUserdata(cam_num)

        self.cam = ic.TIS_CAM()
        self.cam.open(self.cam.GetDevices()[cam_num].decode())

        # Load device state from file
        if self.cam.IsDevValid():
            try:
                file_name = 'configs/{}_config.xml'.format(cam_details[str(self.cam_num)]['name'])
            except:
                print('Could not find a device configuration state .xml file for this camera. Please check camera names.')

            self.cam.LoadDeviceStateFromFile(file_name)
            print('Loaded device state: {}'.format(file_name))

        self.add_filters()

    def add_filters(self):
        if self.rotate != 0:
            h_r = self.cam.CreateFrameFilter(b'Rotate Flip')
            self.cam.AddFrameFilter(h_r)
            self.cam.FilterSetParameter(h_r, b'Rotation Angle', self.rotate)

        h_c = self.cam.CreateFrameFilter(b'ROI')
        self.cam.AddFrameFilter(h_c)
        self.cam.FilterSetParameter(h_c, b'Top', self.crop['top'])
        self.cam.FilterSetParameter(h_c, b'Left', self.crop['left'])
        self.cam.FilterSetParameter(h_c, b'Height', self.crop['height'])
        self.cam.FilterSetParameter(h_c, b'Width', self.crop['width'])
        self.size = (self.crop['width'], self.crop['height'])

    def set_frame_rate(self, fps):
        self.cam.SetFrameRate(fps) #TODO: set 2xfps if in trigger mode

    def set_exposure(self, val):
        val = 1 if val > 1 else val
        val = 0 if val < 0 else val
        self.cam.SetPropertyAbsoluteValue("Exposure", "Value", val)

    def get_exposure(self):
        exposure = [0]
        self.cam.GetPropertyAbsoluteValue("Exposure", "Value", exposure)
        return round(exposure[0], 3)

    def get_image(self):
        self.cam.SnapImage()
        frame = self.cam.GetImageEx()
        return cv2.flip(frame,0)

    def get_image_dimensions(self):
        im = self.get_image()
        height = im.shape[0]
        width = im.shape[1]
        return (width, height)

    def start(self, show_display=1):
        self.cam.SetContinuousMode(0)
        self.cam.StartLive(show_display)

    def close(self):
        self.cam.StopLive()

    def GPout(self):
        self.cam.SetPropertyValue('GPIO', 'GP Out', 1)

    def strobe_on(self):
        self.cam.SetPropertySwitch('Strobe', 'Enable', 1) #1: on
        self.cam.SetPropertyValue('Strobe', 'Mode', 2) #2: exposure
        self.cam.SetPropertySwitch('Strobe', 'Polarity', 1)

    def strobe_off(self):
        self.cam.SetPropertySwitch('Strobe', 'Enable', 0)

    def trigger_on(self):
        self.cam.SetPropertySwitch('Trigger', 'Enable', 1) #1: on
        self.cam.SetPropertySwitch('Trigger', 'Polarity', 1)
        self.cam.SetPropertyAbsoluteValue('Trigger', 'Delay', 5) # in microsecond (min. is 3.1 us)
        self.cam.SetPropertyValue('Trigger', 'IMXLowLatencyMode', 1)
        self.cam.SetPropertyValue('Trigger', 'Exposure Mode', 0) #0: timed (pulse width is exposure time)

    def software_trigger(self):
        self.user_data.image_description = self.cam.GetImageDescription()
        self.user_data.trigger_times.append(time.time())
        self.cam.PropertyOnePush('Trigger', 'Software Trigger')
        return

    def set_frame_ready_callback(self):
        self.cam.SetFrameReadyCallback(callback_function_ptr, self.user_data)
