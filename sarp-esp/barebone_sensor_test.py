import time
import builtins
from machine import Pin, PWM, Timer, WDT
#from stepper import Stepper
from serial import SerialComm
from hcsr04 import HCSR04
from imu import MPU6050
from machine import Pin, I2C
from ADS1115 import *
import _thread
import sys
import gc
import os

ultrasonic1 = HCSR04(2, 3)
ultrasonic2 = HCSR04(4, 5)
ultrasonic3 = HCSR04(6, 7)
ultrasonic4 = HCSR04(8, 9)
bumper1 = Pin(10, Pin.IN, Pin.PULL_DOWN)
bumper2 = Pin(11, Pin.IN, Pin.PULL_DOWN)
bumper3 = Pin(12, Pin.IN, Pin.PULL_DOWN)
bumper4 = Pin(13, Pin.IN, Pin.PULL_DOWN)


while True:
    distance1 = ultrasonic1.distance_cm()
    distance2 = ultrasonic2.distance_cm()
    distance3 = ultrasonic3.distance_cm()
    distance4 = ultrasonic4.distance_cm()
    bump1 = not(bumper1.value())
    bump2 = not(bumper2.value())
    bump3 = not(bumper3.value())
    bump4 = not(bumper4.value())
    print("DISTANCES: "+"{:4.2f}".format(distance1) + " || " + "{:4.2f}".format(distance2)+ " || " + "{:4.2f}".format(distance3)+ " || " + "{:4.2f}".format(distance4))
    print("BUMPERS:" +str(bump1)+ " || " +str(bump2)+ " || " +str(bump3)+ " || " +str(bump4)+ " || " )
    time.sleep(1)

