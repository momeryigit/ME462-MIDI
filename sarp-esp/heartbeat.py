import time
from machine import WDT

class Heartbeat:
    def __init__(self, timeout=4000):
        self.wdt = WDT(timeout=timeout)  # enable it with a timeout of 2s
        self.sent = False
        self._internal_timer = time.ticks_ms()
        self._timeout = timeout/2 #half of WDT timeout
        
    def beat(self):
        #If the time since the last beat is greater than the timeout, send a heartbeat
        if time.ticks_diff(time.ticks_ms(), self._internal_timer) > self._timeout and not self.sent:
            print('h')
            self.sent = True
            self._internal_timer = time.ticks_ms()
    
    def feed(self):
        if self.sent:
            self.wdt.feed()
            self._internal_timer = time.ticks_ms()
            self.sent = False
            print("Im Fed")
        
    def virtual_reset(self):
        if time.ticks_diff(time.ticks_ms(), self._internal_timer) > self._timeout*2 and self.sent == True:
            print("I am resetting")
            time.sleep(0.5)
            self._internal_timer = time.ticks_ms()
            self.sent = False
