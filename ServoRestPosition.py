import sys
import config_serial_servo as css
import time 
from numpy import int8
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Global Variables


id= [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
Kp= 0.1
Kd= 0.05
Servo_Offset=[None]*16

for i in range(16):
    # Accounting for the disconnected arms of the robot
    # only computing the servo rest position of the leg joints
    if (i >= 0 and i <= 4) or (i >= 8 and i <= 12):
        Servo_Offset[i] = css.serial_servo_read_pos(id[i])
    else:
        Servo_Offset[i] = 0
    
    print(Servo_Offset[i])
    
    