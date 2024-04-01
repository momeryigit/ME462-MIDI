import serial

class Serial():
    def __init__(self):
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0)
        except Exception as e:
            print('Failed to initialize serial port.')
            print(str(e))
            
    def write(self, msg):
        self.ser.write(f"{msg}\n".encode('utf-8'))

    def readline(self):
        return self.ser.readline()

