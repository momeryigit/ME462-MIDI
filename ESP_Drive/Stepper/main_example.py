from machine import Pin, UART
import time

class stepper_wheels():
    def __init__(self, dirPinR, stepPinR, dirPinL, stepPinL, stepcount):
        self.dirPinR = Pin(dirPinR, Pin.OUT)
        self.stepPinR = Pin(stepPinR, Pin.OUT)
        self.dirPinL = Pin(dirPinL, Pin.OUT)
        self.stepPinL = Pin(stepPinL, Pin.OUT)
        self.stepcount = stepcount
        self.dirPin.off()
        self.stepPin.off()
    
    def move(self, leftdir, rightdir, step, delay):
        self.dirPinR.value(rightdir)
        self.dirPinL.value(leftdir)
        for i in range(0,step):
            self.stepPinR.on()
            self.stepPinL.on()
            time.sleep(delay/1000)
            self.stepPinR.off()
            self.stepPinL.off()
            time.sleep(delay/1000)
    

    
class serial():
    def __init__(self, uart_id=0, baud_rate=9600):
        self.uart = UART(uart_id, baud_rate)
        self.uart.init(baud_rate, bits=8, parity=None, stop=1)
    def read(self):
        if self.uart.any():
            return self.uart.read().decode('utf-8')
    def write(self, data):
        self.uart.write(data.encode('utf-8'))

    def check_conn(self):
        self.write('AT')
        while True:
            if self.read() == 'OK':
                print('Connected')
                break
            else:
                print('Not connected')
                time.sleep(1)
                self.write('AT')


while True:
    try:
        ser = serial()
        ser.check_conn()
        break
    except:
        print('Error')
        time.sleep(1)
        continue

while True:
    data = ser.read()
    if data:
        print(data)
        if data == 'F':
            break
    time.sleep(0.1)





    
    
        