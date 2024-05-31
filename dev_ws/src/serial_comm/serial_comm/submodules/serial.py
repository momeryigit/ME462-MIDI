import serial

class Serial():
    def __init__(self, port = '/dev/ttyUSB0', baud = 115200):
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
        except Exception as e:
            print('Failed to initialize serial port.')
            print(str(e))
            
    def write(self, msg):
        self.ser.write(f"{msg}\n".encode('utf-8'))

    def read(self):
        return self.ser.readline().decode("utf-8").strip()

