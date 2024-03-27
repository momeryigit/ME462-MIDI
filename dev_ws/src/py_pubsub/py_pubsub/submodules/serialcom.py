import serial
import time

class SerialCom:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=1)

    def send(self, msg):
        self.ser.write(msg.encode())

    def read(self):
            return self.ser.readline().decode()
    def check_conn(self):
        while True:
            inc = ''
            if self.ser.in_waiting > 0:
                inc = self.ser.readline().decode()
            else:
                print('No data')

            if inc == 'AT':
                self.ser.write('OK'.encode())
                break
    def close(self):
        self.ser.close()
