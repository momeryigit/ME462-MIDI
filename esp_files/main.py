import time
import builtins
import uselect
import sys
from machine import Pin, PWM, Timer
from stepper import Stepper

class SerialComm:
    def __init__(self):
        self.serialPoll = uselect.poll()
        self.serialPoll.register(sys.stdin, uselect.POLLIN)
        self.toSend = []

    def read_message(self):
        return(sys.stdin.readline() if self.serialPoll.poll(0) else None)

    def send_message(self, message):
        print(message)
    
    def read_parse(self):
        message = self.read_message()
        if message:
            return message.split(' ') 
        else:
            return None
    def queue_mes(self, message):
        self.toSend.append(message)
        
    def send_queue(self):
        for msg in self.toSend:
            self.send_message(msg)
        self.toSend = []
  
beats = 0
def heartbeat(serobj):
    global beats
    beats += 1
    serobj.queue_mes('h ' + str(beats))
    
def toggle(mypin):
    mypin.value(not mypin.value())

def main():
    s1 = Stepper(19,18,steps_per_rev=200,speed_sps=10)
    s1.free_run(1)
    ser = SerialComm()
    tim = Timer()
    tim2 = Timer()
    
    tim.init(freq=0.5, mode=Timer.PERIODIC, callback=lambda t: heartbeat(ser))
    led = Pin(20, Pin.OUT)
    tim2.init(freq=1, mode=Timer.PERIODIC, callback=lambda t: toggle(led))
    s1.enable(0)

    while True:
        msg = ser.read_parse()
        if msg:
            print(msg)
            if msg[0] == 's':
                s1.enable(1)
                s1.free_run(0)
                s1.speed(int(msg[1]))
                s1.free_run(int(msg[2]))
                print(s1.steps_per_sec)
            if msg[0]=='q':
                s1.free_run(0)
                s1.enable(0)
                break
        ser.send_queue()
        
            


        
    
if __name__ == '__main__':
    main()