
import time
import builtins
import uselect
import sys
from wificon import connect_to_wifi
from machine import Pin, PWM, Timer
from stepper import Stepper


# class Stepper:
#     def __init__(self, direction_pin, step_pin, freq=500):
#         self.direction_pin = Pin(direction_pin, Pin.OUT)
#         self.step_pin = PWM(Pin(step_pin))
#         self.steps = 0
#         self.timer = Timer()

#     def set_direction(self, direction):
#         self.direction_pin.value(direction)


#     def start(self, freq=500, duty=32768):
#         self.step_pin.freq(freq)
#         self.step_pin.duty_u16(duty)
#         self.timer.init(mode=Timer.PERIODIC, freq=self.step_pin.freq(), callback=self._step)

#     def stop(self):
#         self.step_pin.duty_u16(0)
#         self.steps = 0
#         self.timer.deinit()

#     def _step(self, timer):
#         self.steps += 1



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
    connect_to_wifi('sarphotspot', 'lokomotif')
    s1 = Stepper(19,18,steps_per_rev=200,speed_sps=10)
    s1.free_run(1)
    ser = SerialComm()
    tim = Timer(0)
    tim2 = Timer(1)
    
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