# -*- coding: utf-8 -*-
"""
Created on Sun Feb  4 20:29:15 2018

@author: adity
"""

#from Tkinter import *
import RPi.GPIO as GPIO
#import time

min_dc = 3      #y1
min_ang = 0     #x1
max_dc = 11     #y2
max_ang = 180   #x2
num = float((max_dc-min_dc))
den = float((max_ang-min_ang))
m =float(num/den)
print('m1',m)

'''
m = float((y2-y2)/(x2-x1))  ---slope or scaling factor

y-y1 = m(x-x1)
y = m(x-x1) + y1
New duty cycle = m * (desired_ang - min_ang) + min_dc
New duty cycle = m * (desired_ang - 0) + min_dc

New duty cycle = m * (desired_ang) + min_dc
    
'''


GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

pwm = GPIO.PWM(18, 53)

try:
    pwm.start(7.5)
    desired_ang = 90


    
    while True:
        G= input()
        T = int(G)
        if T == 0:
            pwm.stop()
            GPIO.cleanup()
            break
        elif (T != 0):
            print('Value',G)
            
            if (T==4):
                
                
                if (desired_ang > min_ang):
                    print('Decrement Possible')
                    desired_ang = desired_ang - 15
                    print('Desired_Angle=', desired_ang)
                    
                    new_dc = m * float(desired_ang) + float(min_dc)
                    print('m',m)
                    print(new_dc)
                    
                    pwm.ChangeDutyCycle(new_dc)
                    
                elif (desired_ang == min_ang):
                    print('Already at 0 degrees')
                
                
                #p.ChangeDutyCycle(7.5)
            elif (T==6):
                
                
                if (desired_ang < max_ang):
                    print('Increment Possible')
                    desired_ang = desired_ang + 15
                    print('Desired_Angle=', desired_ang)
                    
                    new_dc = m * float(desired_ang) + float(min_dc)
                    print(new_dc)
                    pwm.ChangeDutyCycle(new_dc)
                    
                    
                elif (desired_ang == max_ang):
                    print('Already at 0 degrees')
                    
except KeyboardInterrupt:
    print('1')
    pwm.stop()
    GPIO.cleanup()
    
    
