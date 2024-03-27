import sys
import uselect
import time
from machine import Pin

class stepper_wheels():
    def __init__(self, dirPinR, stepPinR, dirPinL, stepPinL, stepcount, delay):
        self.dirPinR = Pin(dirPinR, Pin.OUT)
        self.stepPinR = Pin(stepPinR, Pin.OUT)
        self.dirPinL = Pin(dirPinL, Pin.OUT)
        self.stepPinL = Pin(stepPinL, Pin.OUT)
        self.stepcount = stepcount
        self.dirPinR.off()
        self.stepPinR.off()
        self.dirPinL.off()
        self.stepPinL.off()
        self.timer = time.ticks_ms()
        self.delay = delay
        
        
    def move(self, leftdir, rightdir):
        rightdir = not(rightdir)
        self.dirPinR.value(rightdir)
        self.dirPinL.value(leftdir)
        self.stepPinR.on()
        self.stepPinL.on()            
        while time.ticks_ms() - self.timer < self.delay:
            pass
        self.timer = time.ticks_ms()
        self.stepPinR.off()
        self.stepPinL.off()
        while time.ticks_ms() - self.timer < self.delay:
            pass
    def stop(self):
        self.dirPinR.off()
        self.stepPinR.off()
        self.dirPinL.off()
        self.stepPinL.off()

def readSerial():
    """
    reads a single character over serial.

    :return: returns the character which was read, otherwise returns None
    """
    return(sys.stdin.read(1) if serialPoll.poll(0) else None)

serialPoll = uselect.poll()
serialPoll.register(sys.stdin, uselect.POLLIN)

motors = stepper_wheels(16, 26, 27, 25, 200, 3)

while True:
    # continuously read commands over serial and handle them
    message = readSerial()
    if not message == None:
        while True:
            if message == "w":
                while True:
                    motors.move(1, 1)
                    message = readSerial()
                    if message != None:
                        break
            elif message == "s":
                while True:
                    motors.move(0, 0)
                    message = readSerial()
                    if message != None:
                        break
            elif message == "a":
                while True:
                    motors.move(0, 1)
                    message = readSerial()
                    if message != None:
                        break
            elif message == "d":
                while True:
                    motors.move(1, 0)
                    message = readSerial()
                    if message != None:
                        break
            elif message == "e":
                motors.stop()
                while True:
                    message = readSerial()
                    if message != None:
                        break    
            
    
        
