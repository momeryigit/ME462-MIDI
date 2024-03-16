import serial
import time

def burdayim():
    print("I am here")

class SerialCom:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=1)

    def send(self, msg):
        self.ser.write(msg.encode())

    def receive(self):
        if self.ser.in_waiting > 0:
            return self.ser.readline().decode()

    def close(self):
        self.ser.close()
