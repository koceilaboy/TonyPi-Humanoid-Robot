import sys
import config_serial_servo as css
import time
import numpy as np
from numpy import int8
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Global Variables

# Servo Ids
id= [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
# PD Controller Gains
Kp= 0.2
Kd= 0.001
# Servos default position
Servo_rest_pos= np.array([498,388,498,593,499,0,0,0,498,611,501,405,499,0,0,0])
Servo_rest_pos_deg= Servo_rest_pos * 180/1000

#Initializing arrays
offset = [0]*16
pos_deg= [0]*16
input= [0]*16
u= [0]*16
ep= [0]*16
dt= [0]*16
end= [0]*16
start= [0.001]*16


def main():
    
    desired_pos = np.array([0,0,0,0,0,  0,0,0,  0,0,0,0,0,  0,0,0]) # degrees
    last_error = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] # D term of the PD controlelr
    
    
    while True:
        for i in range(16):
            if (i >= 8 and i <= 12) or (i >= 0 and i <= 4): 
                start[i] = time.time() # start timing
                
                # Servo Input Offset
                offset[i] = (desired_pos[i] * 1000/180) + Servo_rest_pos[i]
                
                # Position measurement
                pos = css.serial_servo_read_pos(id[i]) # Servo position in servo units
                pos_deg[i]= (pos*180)/1000 - Servo_rest_pos_deg[i] # converting from servo units to degrees
                
                # OD Condtroller
                EP_servo =(offset[i] - pos)
                ed = (EP_servo - last_error[i])
                last_error[i] = EP_servo
                u[i] =  Kp*EP_servo + Kd*ed
                
                # Turn off esrvo Input when error is small enough 
                if EP_servo <= 3 and EP_servo >= -3: 
                    u[i]=0
                
                # Servo Input 
                input[i] = int( u[i] + offset[i])
                
                # Limitting Servo input to rmeain iwthin th eexpected range (0~1000)
                if input[i] >= 1000:
                    input[i] = 1000
                elif input[i] <= 0:
                    input[i] = 0

                # Sending serial input to the specific servo with a given ID
                css.serial_servo_set_pos(id[i], input[i], 0) 
                
                # Time between samples
                # Note this is not used in this code, but can be uesd in the D term of the PD controller
                end[i] = time.time() # end[i] time 
                dt[i] = end[i]-start[i] # sampling time 
                
                # Required wait time otherwise it may crash
                # Application dependent, and found using trail and error
                time.sleep(0.0004)
                # print(pos_deg[i], desired_pos[i], input[i], u[i])
         
         
if __name__ == "__main__": 
    main() 

        
    