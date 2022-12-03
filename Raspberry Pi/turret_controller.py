from turret_pwm import TURRET_PWM
from turret_imu import TURRET_IMU
from turret_camera import TURRET_CAMERA

import cv2
import numpy as np

FRACTIONAL_UPDATE = 0.05

pwm = TURRET_PWM()
imu = TURRET_IMU()
cam = TURRET_CAMERA()

yaw_val = 0
pitch_val = 0

while True:
    # Get data from camera
    tag_in_frame = cam.id_target_tag()
    
    # If the target tag is in the frame
    if tag_in_frame:
        
        # Calculate center-to-center distance
        dx, dy = cam.get_distance_from_center()
        
        pitch_val = FRACTIONAL_UPDATE*dx + (1-FRACTIONAL_UPDATE)*pitch_val
        yaw_val = FRACTIONAL_UPDATE*dx + (1-FRACTIONAL_UPDATE)*yaw_val
                
        pwm.set_pitch(pitch_val)
        pwm.set_yaw(yaw_val)
       
    cam.show_frame()
    
    if cv2.waitKey(1) == ord('q'): 
        break
    
cam.shutdown()
