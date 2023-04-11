"""
Camera Control
Copyright M. Mathis Lab
Written by  Gary Kane - https://github.com/gkane26
post-doctoral fellow @ the Adaptive Motor Control Lab
https://github.com/AdaptiveMotorControlLab

create json parameter file
"""

import os
import json
import ctypes
import tisgrabber_tis as tis

path = os.path.dirname(os.path.realpath(__file__))
out = os.path.normpath(path + '/camera_details.json')

# Crop, rotation, and exposure are default parameters. Can be changed in the GUI.
# NOTE: you need to know how cameras are ordered in your system (check in IC Capture) e.g. camera 0 is lateral
cam_0 = {'name' : 'ephys_1_lateral',
        'crop' : {'top' : 0, 'left' : 0, 'height' : 540, 'width' : 720},
        'rotate' : 0,
        'exposure' : 0.002,
        'output_dir' : r'C:/Users/bisi/Desktop/video'}

cam_1 = {'name' : 'ephys_1_top',
        'crop' : {'top' : 0, 'left' : 0, 'height' : 540, 'width' : 720},
        'rotate' : 0,
        'exposure' : 0.002,
        'output_dir' : r'C:/Users/bisi/Desktop/video'}

subs = ['ABXXX'] # optional, can manually enter subject for each session.

#labview = ['Dev1/port0/line0'] # optional, can manually enter for each session

n_cams = 2
cam_list = [cam_0, cam_1]
details = {'cams' : n_cams,
           '0' : cam_0,
           '1' : cam_1,
           'subjects' : subs,
           #'labview' : labview
           }

# Write camera details from GUI
with open(out, 'w') as handle:
    json.dump(details, handle)

# Save device state to file
ic = ctypes.cdll.LoadLibrary("./tisgrabber_x64.dll")
tis.declareFunctions(ic)
ic.IC_InitLibrary(0)

for cam in cam_list:
    print('Configuring {}'.format(cam['name']))
    hGrabber = ic.IC_ShowDeviceSelectionDialog(None)

    if(ic.IC_IsDevValid(hGrabber)):
        config_file_name = 'configs/{}_config.xml'.format(cam['name'])
        ic.IC_SaveDeviceStateToFile(hGrabber, tis.T(config_file_name))
    else:
        ic.IC_MsgBox(tis.T("No device opened"), tis.T("Simple Live Video"))

    ic.IC_ReleaseGrabber(hGrabber)
